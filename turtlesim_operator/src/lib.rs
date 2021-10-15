//
// Copyright (c) 2017, 2021 ADLINK Technology Inc.
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ADLINK zenoh team, <zenoh@adlink-labs.tech>
//

use cxx::UniquePtr;
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    downcast_mut, zf_data, Node, NodeOutput, Operator, SerDeData, State, Token, TokenAction,
    ZFError, ZFResult, DowncastAny, Data
};

extern crate zenoh_flow;

#[cxx::bridge(namespace = "zenoh::flow")]
pub mod ffi {

    pub struct geometry_msgs_Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    pub struct geometry_msgs_Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    pub struct geometry_msgs_Twist {
        pub linear: geometry_msgs_Vector3,
        pub angular: geometry_msgs_Vector3,
    }
    
    pub struct Context {
        pub mode: usize,
    }

    pub struct Configuration {
        pub key: String,
        pub value: String,
    }

    pub struct Input {
        pub port_id: String,
        pub data: Vec<u8>,
        pub timestamp: u64,
    }

    pub struct Output {
        pub port_id: String,
        pub data: Vec<u8>,
    }

    pub enum TokenStatus {
        Pending,
        Ready,
        DeadlineMiss,
    }

    pub enum TokenAction {
        Consume,
        Drop,
        Keep,
        Postpone,
        Wait,
    }

    pub struct Token {
        pub status: TokenStatus,
        pub action: TokenAction,
        pub port_id: String,
        pub data: Vec<u8>,
        pub timestamp: u64,
    }

    unsafe extern "C++" {
        include!("operator.hpp");

        type State;

        fn initialize(configuration: &Vec<Configuration>) -> UniquePtr<State>;

        fn input_rule(
            context: &mut Context,
            state: &mut UniquePtr<State>,
            tokens: &mut Vec<Token>,
        ) -> Result<bool>;

        fn run(
            context: &mut Context,
            state: &mut UniquePtr<State>,
            twist: &geometry_msgs_Twist,
        ) -> Result<geometry_msgs_Twist>;
    }
}

impl Data for ffi::geometry_msgs_Twist {
    fn try_serialize(&self) -> zenoh_flow::ZFResult<Vec<u8>> {
        Ok(Vec::new())
    }
}

impl DowncastAny for ffi::geometry_msgs_Twist {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl Debug for ffi::geometry_msgs_Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("geometry_msgs__msg__Vector3")
        .field("x", &self.x)
        .field("y", &self.y)
        .field("z", &self.z)
        .finish()
    }
}

impl Debug for ffi::geometry_msgs_Quaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("geometry_msgs__msg__Quaternion")
        .field("x", &self.x)
        .field("y", &self.y)
        .field("z", &self.z)
        .field("w", &self.w)
        .finish()
    }
}

impl Debug for ffi::geometry_msgs_Twist {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("geometry_msgs__msg__Twist")
        .field("linear", &self.linear)
        .field("angular", &self.angular)
        .finish()
    }
}
unsafe impl Send for ffi::State {}
unsafe impl Sync for ffi::State {}

pub struct StateWrapper {
    pub state: UniquePtr<ffi::State>,
}

impl State for StateWrapper {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl Debug for StateWrapper {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        todo!()
    }
}



pub struct TwistWrapper {
    pub twist: ffi::geometry_msgs_Twist,
}

// impl Data for TwistWrapper {
//     fn try_serialize(&self) -> zenoh_flow::ZFResult<Vec<u8>> {
//         // Ok(self.0.as_bytes().to_vec())
//         Ok(vec![0])
//     }
// }

impl Debug for TwistWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TwistWrapper")
        .field("twist", &self.twist)
        .finish()
    }
}

impl DowncastAny for TwistWrapper {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}


impl From<&mut zenoh_flow::Context> for ffi::Context {
    fn from(context: &mut zenoh_flow::Context) -> Self {
        Self { mode: context.mode }
    }
}

impl ffi::Token {
    pub fn from_token(token: &Token, port_id: &str) -> ZFResult<Self> {
        match token {
            Token::NotReady => Ok(Self {
                status: ffi::TokenStatus::Pending,
                action: ffi::TokenAction::Wait,
                port_id: port_id.to_string(),
                data: Vec::new(),
                timestamp: 0,
            }),

            Token::Ready(token) => {
                let data = match &token.data.data {
                    SerDeData::Serialized(ser) => ser.as_ref().clone(),
                    SerDeData::Deserialized(de) => de.try_serialize()?,
                };

                Ok(Self {
                    status: ffi::TokenStatus::Ready,
                    action: ffi::TokenAction::Consume,
                    port_id: port_id.to_string(),
                    data,
                    timestamp: token.data.timestamp.get_time().as_u64(),
                })
            }
        }
    }
}

impl From<TokenAction> for ffi::TokenAction {
    fn from(action: TokenAction) -> Self {
        match action {
            TokenAction::Consume => ffi::TokenAction::Consume,
            TokenAction::Drop => ffi::TokenAction::Drop,
            TokenAction::KeepRun => ffi::TokenAction::Keep,
            TokenAction::Keep => ffi::TokenAction::Keep,
            TokenAction::Wait => ffi::TokenAction::Wait,
        }
    }
}

impl ffi::Input {
    fn from_data_message(
        port_id: &str,
        data_message: &zenoh_flow::runtime::message::DataMessage,
    ) -> ZFResult<Self> {
        let data = match &data_message.data {
            SerDeData::Serialized(ser) => ser.as_ref().clone(),
            SerDeData::Deserialized(de) => de.try_serialize()?,
        };

        Ok(Self {
            port_id: port_id.to_string(),
            data,
            timestamp: data_message.timestamp.get_time().as_u64(),
        })
    }
}

static TWIST: &str = "twist";

pub struct TurtlesimOperator;

impl Node for TurtlesimOperator {
    fn initialize(
        &self,
        configuration: &Option<std::collections::HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::State> {
        let cxx_configuration = match configuration {
            Some(config) => config
                .iter()
                .map(|(key, value)| ffi::Configuration {
                    key: key.clone(),
                    value: value.clone(),
                })
                .collect(),
            None => vec![],
        };

        let state = {
            #[allow(unused_unsafe)]
            unsafe {
                ffi::initialize(&cxx_configuration)
            }
        };
        Box::new(StateWrapper { state })
    }

    fn clean(&self, _state: &mut Box<dyn zenoh_flow::State>) -> ZFResult<()> {
        Ok(())
    }
}

impl Operator for TurtlesimOperator {
    fn input_rule(
        &self,
        context: &mut zenoh_flow::Context,
        dyn_state: &mut Box<dyn zenoh_flow::State>,
        tokens: &mut HashMap<zenoh_flow::PortId, zenoh_flow::Token>,
    ) -> zenoh_flow::ZFResult<bool> {
        let wrapper = downcast_mut!(StateWrapper, dyn_state).unwrap();
        let res_cxx_tokens: Result<Vec<ffi::Token>, ZFError> = tokens
            .iter()
            .map(|(port_id, token)| ffi::Token::from_token(token, port_id))
            .collect();
        let mut cxx_tokens = res_cxx_tokens?;
        let mut cxx_context = ffi::Context::from(context);

        {
            #[allow(unused_unsafe)]
            unsafe {
                ffi::input_rule(&mut cxx_context, &mut wrapper.state, &mut cxx_tokens)
                    .map_err(|_| ZFError::GenericError)
            }
        }
    }

    fn run(
        &self,
        context: &mut zenoh_flow::Context,
        dyn_state: &mut Box<dyn zenoh_flow::State>,
        inputs: &mut HashMap<zenoh_flow::PortId, zenoh_flow::runtime::message::DataMessage>,
    ) -> ZFResult<HashMap<zenoh_flow::PortId, SerDeData>> {
        let mut cxx_context = ffi::Context::from(context);
        let wrapper = downcast_mut!(StateWrapper, dyn_state).unwrap();
        let (_, pose_with_cov_stamped_wrapper) = autocore_get_input!(TwistWrapper, String::from(TWIST), inputs)?;

        let cxx_output = {
            #[allow(unused_unsafe)]
            unsafe {
                ffi::run(&mut cxx_context, &mut wrapper.state, &pose_with_cov_stamped_wrapper.twist)
                    .map_err(|_| ZFError::GenericError)?
            }
        };

        let mut result: HashMap<zenoh_flow::PortId, SerDeData> =
            HashMap::with_capacity(1);
        result.insert(
            TWIST.into(),
            zf_data!(cxx_output),
        );

        Ok(result)
    }

    fn output_rule(
        &self,
        _context: &mut zenoh_flow::Context,
        _dyn_state: &mut Box<dyn zenoh_flow::State>,
        outputs: HashMap<zenoh_flow::PortId, SerDeData>,
    ) -> ZFResult<HashMap<zenoh_flow::PortId, zenoh_flow::NodeOutput>> {
        let mut results = HashMap::with_capacity(outputs.len());
        // NOTE: default output rule for now.
        for (port_id, data) in outputs {
            results.insert(port_id, NodeOutput::Data(data));
        }

        Ok(results)
    }
}

zenoh_flow::export_operator!(register);

fn register() -> ZFResult<Arc<dyn Operator>> {
    Ok(Arc::new(TurtlesimOperator) as Arc<dyn Operator>)
}

#[macro_export]
macro_rules! autocore_get_input {
    ($ident : ident, $index : expr, $map : expr) => {
        match $map.get_mut::<str>(&$index) {
            Some(data_message) => match &data_message.data {
                zenoh_flow::SerDeData::Deserialized(de) => {
                    match zenoh_flow::downcast!($ident, de) {
                        Some(data) => Ok((data_message.timestamp.clone(), data.clone())),
                        None => Err(zenoh_flow::types::ZFError::InvalidData($index)),
                    }
                }
                zenoh_flow::SerDeData::Serialized(_) => {
                    Err(zenoh_flow::types::ZFError::InvalidData($index))
                }
            },
            None => Err(zenoh_flow::types::ZFError::MissingInput($index)),
        }
    };
}