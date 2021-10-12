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
use std::{collections::HashMap, fmt::Debug, sync::Arc};
use zenoh_flow::{
    downcast_mut, runtime::message::DataMessage, Component, Context, Data, DowncastAny, InputRule,
    PortId, SerDeData, Sink, State, Token, TokenAction, ZFError, ZFResult,
};

extern crate zenoh_flow;

#[cxx::bridge(namespace = "zenoh::flow")]
pub mod ffi {
    pub struct Context {
        pub mode: usize,
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

    pub struct Input {
        pub port_id: String,
        pub data: Vec<u8>,
        pub timestamp: u64,
    }

    pub struct Output {
        pub port_id: String,
        pub data: Vec<u8>,
    }

    pub struct Data {
        pub bytes: Vec<u8>,
    }

    pub struct Configuration {
        pub key: String,
        pub value: String,
    }

    pub struct ConfigurationMap {
        pub map: Vec<Configuration>,
    }
    unsafe extern "C++" {
        include!("sink.hpp");

        type State;

        fn initialize(configuration: &ConfigurationMap) -> UniquePtr<State>;

        fn input_rule(
            context: &mut Context,
            state: &mut UniquePtr<State>,
            tokens: &mut Vec<Token>,
        ) -> Result<bool>;

        fn run(
            context: &mut Context,
            state: &mut UniquePtr<State>,
            inputs: Vec<Input>,
        ) -> Result<()>;
    }
}
impl From<HashMap<String, String>> for ffi::ConfigurationMap {
    fn from(configuration: HashMap<String, String>) -> Self {
        ffi::ConfigurationMap {
            map: configuration
                .iter()
                .map(|(key, value)| ffi::Configuration {
                    key: key.clone(),
                    value: value.clone(),
                })
                .collect(),
        }
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

impl ffi::Data {
    pub fn new(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }
}

impl Debug for ffi::Data {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").field("bytes", &self.bytes).finish()
    }
}

impl DowncastAny for ffi::Data {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl Data for ffi::Data {
    fn try_serialize(&self) -> ZFResult<Vec<u8>> {
        Ok(self.bytes.clone())
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

pub struct MySink;

impl Component for MySink {
    fn initialize(
        &self,
        configuration: &Option<std::collections::HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::State> {
        let configuration = match configuration {
            Some(config) => ffi::ConfigurationMap::from(config.clone()),
            None => ffi::ConfigurationMap { map: Vec::new() },
        };

        let state = {
            #[allow(unused_unsafe)]
            unsafe {
                ffi::initialize(&configuration)
            }
        };
        Box::new(StateWrapper { state })
    }

    fn clean(&self, _state: &mut Box<dyn State>) -> ZFResult<()> {
        Ok(())
    }
}

impl InputRule for MySink {
    fn input_rule(
        &self,
        context: &mut zenoh_flow::Context,
        dyn_state: &mut Box<dyn State>,
        tokens: &mut HashMap<PortId, zenoh_flow::Token>,
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
}

#[async_trait]
impl Sink for MySink {
    async fn run(
        &self,
        context: &mut Context,
        dyn_state: &mut Box<dyn State>,
        inputs: &mut HashMap<PortId, DataMessage>,
    ) -> ZFResult<()> {
        let mut cxx_context = ffi::Context::from(context);
        let wrapper = downcast_mut!(StateWrapper, dyn_state).unwrap();
        let result_cxx_inputs: Result<Vec<ffi::Input>, ZFError> = inputs
            .iter()
            .map(|(port_id, data_message)| ffi::Input::from_data_message(port_id, data_message))
            .collect();
        let cxx_inputs = result_cxx_inputs?;

        {
            #[allow(unused_unsafe)]
            unsafe {
                Ok(ffi::run(&mut cxx_context, &mut wrapper.state, cxx_inputs)
                    .map_err(|_| ZFError::GenericError)?)
            }
        }
    }
}

zenoh_flow::export_sink!(register);

fn register() -> ZFResult<Arc<dyn Sink>> {
    Ok(Arc::new(MySink) as Arc<dyn Sink>)
}
