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

use async_trait::async_trait;
use cxx::UniquePtr;
use std::{fmt::Debug, sync::Arc};
use zenoh_flow::{
    downcast_mut, runtime::message::DataMessage, Context, Node, SerDeData, Sink, State, ZFError,
    ZFResult, DowncastAny
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
        pub data: Vec<u8>,
        pub timestamp: u64,
    }

    unsafe extern "C++" {
        include!("sink.hpp");

        type State;

        fn initialize(configuration: &Vec<Configuration>) -> UniquePtr<State>;

        // fn run(
        //     context: &mut Context,
        //     state: &mut UniquePtr<State>,
        //     input: Input,
        // ) -> Result<()>;
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

impl From<&mut zenoh_flow::Context> for ffi::Context {
    fn from(context: &mut zenoh_flow::Context) -> Self {
        Self { mode: context.mode }
    }
}

impl ffi::Input {
    fn from_data_message(
        data_message: &zenoh_flow::runtime::message::DataMessage,
    ) -> ZFResult<Self> {
        let data = match &data_message.data {
            SerDeData::Serialized(ser) => ser.as_ref().clone(),
            SerDeData::Deserialized(de) => de.try_serialize()?,
        };

        Ok(Self {
            data,
            timestamp: data_message.timestamp.get_time().as_u64(),
        })
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

static TWIST: &str = "twist";

pub struct TurtlesimSink;

impl Node for TurtlesimSink {
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

    fn clean(&self, _state: &mut Box<dyn State>) -> ZFResult<()> {
        Ok(())
    }
}

#[async_trait]
impl Sink for TurtlesimSink {
    async fn run(
        &self,
        _context: &mut Context,
        _dyn_state: &mut Box<dyn State>,
        input: DataMessage,
    ) -> ZFResult<()> {
        // let mut cxx_context = ffi::Context::from(context);
        // let wrapper = downcast_mut!(StateWrapper, dyn_state).unwrap();
        let (_, pose_with_cov_stamped_wrapper) = autocore_sink_get_input!(TwistWrapper, String::from(TWIST), input)?;
        println!("{:?}", pose_with_cov_stamped_wrapper);
        Ok(())
    }
}

zenoh_flow::export_sink!(register);

fn register() -> ZFResult<Arc<dyn Sink>> {
    Ok(Arc::new(TurtlesimSink) as Arc<dyn Sink>)
}


#[macro_export]
macro_rules! autocore_sink_get_input {
    ($ident : ident, $index : expr, $data_message : expr) => {
        match &$data_message.data {
                zenoh_flow::SerDeData::Deserialized(de) => {
                    match zenoh_flow::downcast!($ident, de) {
                        Some(data) => Ok(($data_message.timestamp.clone(), data.clone())),
                        None => Err(zenoh_flow::types::ZFError::InvalidData($index)),
                    }
                }
                zenoh_flow::SerDeData::Serialized(_) => {
                    Err(zenoh_flow::types::ZFError::InvalidData($index))
                }
            };
        }
}