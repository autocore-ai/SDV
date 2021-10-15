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
use zenoh_flow::Context;
use zenoh_flow::{
    downcast_mut, zf_data, Data, DowncastAny, Node, SerDeData, Source, State, ZFError, ZFResult,
};

extern crate zenoh_flow;

#[cxx::bridge(namespace = "zenoh::flow")]
pub mod ffi {
    pub struct GeometryMsgsVector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    pub struct GeometryMsgsTwist {
        pub linear: GeometryMsgsVector3,
        pub angular: GeometryMsgsVector3,
    }

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
        include!("source.hpp");

        type State;

        fn initialize(configuration: &ConfigurationMap) -> UniquePtr<State>;

        fn run(context: &mut Context, state: &mut UniquePtr<State>) -> GeometryMsgsTwist;
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

impl DowncastAny for ffi::GeometryMsgsTwist {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_mut_any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl Debug for ffi::GeometryMsgsVector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GeometryMsgsVector3")
            .field("x", &self.x)
            .field("y", &self.y)
            .field("z", &self.z)
            .finish()
    }
}

impl Debug for ffi::GeometryMsgsTwist {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GeometryMsgsTwist")
            .field("linear", &self.linear)
            .field("angular", &self.angular)
            .finish()
    }
}

impl Data for ffi::GeometryMsgsTwist {
    fn try_serialize(&self) -> std::result::Result<std::vec::Vec<u8>, zenoh_flow::ZFError> {
        Ok(vec![0])
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
pub struct TurtlesimSource;

impl Node for TurtlesimSource {
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

#[async_trait]
impl Source for TurtlesimSource {
    async fn run(
        &self,
        context: &mut Context,
        dyn_state: &mut Box<dyn State>,
    ) -> ZFResult<SerDeData> {
        let mut cxx_context = ffi::Context::from(context);
        let wrapper = downcast_mut!(StateWrapper, dyn_state).unwrap();
        let cxx_output = {
            #[allow(unused_unsafe)]
            unsafe {
                ffi::run(&mut cxx_context, &mut wrapper.state)
            }
        };
        Ok(zf_data!(cxx_output))
    }
}

zenoh_flow::export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(TurtlesimSource) as Arc<dyn Source>)
}
