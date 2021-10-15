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
    downcast_mut, zf_data_raw, zf_data, Node, Data, DowncastAny, 
    PortId, SerDeData, Source, State, ZFResult,
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

        fn run(context: &mut Context, state: &mut UniquePtr<State>) -> Result<Vec<Output>>;

        fn Data() -> geometry_msgs_Twist;
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

impl Data for ffi::geometry_msgs_Twist {
    fn try_serialize(&self) -> std::result::Result<std::vec::Vec<u8>, zenoh_flow::ZFError> {
        todo!()
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
        _context: &mut zenoh_flow::Context,
        _dyn_state: &mut Box<dyn zenoh_flow::State>,
    ) -> ZFResult<SerDeData> {
        
        #[allow(unused_unsafe)]
        unsafe {
            let geometry_msgs_twist = ffi::Data();
            Ok(zf_data!(geometry_msgs_twist))
        }

    }
}

zenoh_flow::export_source!(register);

fn register() -> ZFResult<Arc<dyn Source>> {
    Ok(Arc::new(TurtlesimSource) as Arc<dyn Source>)
}
