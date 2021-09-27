use async_std::sync::Arc;
// use autocxx::include_cpp;
use std::collections::HashMap;
use std::thread;
use zenoh_flow::runtime::message::ZFDataMessage;
use zenoh_flow::{
    default_input_rule, default_output_rule, export_operator, get_input, types::ZFResult, zf_data,
    zf_empty_state, Token, ZFComponent, ZFComponentInputRule, ZFComponentOutput,
    ZFComponentOutputRule, ZFContext, ZFDataTrait, ZFOperatorTrait, ZFStateTrait,
};
use zenoh_flow_types::{ZFString, ZFUsize, ZFU64};

// include_cpp! {
//     #include "api.hpp"
//     safety!(unsafe)
//     generate!("Api")
//     generate!("GetApi")
// }

// static mut API: *mut ffi::Api = std::ptr::null_mut();

struct OperatorSimplePlanningSimulator;

static LINK_ID_TICK: &str = "tick";
static LINK_ID_INPUT1: &str = "input1";
static LINK_ID_OUTPUT: &str = "output";

impl ZFComponentInputRule for OperatorSimplePlanningSimulator {
    fn input_rule(
        &self,
        _context: &mut ZFContext,
        state: &mut Box<dyn ZFStateTrait>,
        inputs: &mut HashMap<String, Token>,
    ) -> ZFResult<bool> {
        let tick_token = inputs.get(LINK_ID_TICK);
        let input1_token = inputs.get(LINK_ID_INPUT1);

        let mut tick_flag = false;
        let mut input1_flag = false;
        if let Some(Token::Ready(_)) = input1_token {
            input1_flag = true;
        };
        if let Some(Token::Ready(_)) = tick_token {
            tick_flag = true;
        };

        if (!tick_flag && input1_flag) || (!tick_flag && !input1_flag){
            return Ok(false)
        } else if !input1_flag && tick_flag{
            inputs.remove(LINK_ID_INPUT1);
        }

        Ok(true)
    }
}

impl ZFComponent for OperatorSimplePlanningSimulator {
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn ZFStateTrait> {
        // unsafe { API = ffi::GetApi() as *mut ffi::Api };
        zf_empty_state!()
    }

    fn clean(&self, _state: &mut Box<dyn ZFStateTrait>) -> ZFResult<()> {
        Ok(())
    }
}

impl ZFOperatorTrait for OperatorSimplePlanningSimulator {
    fn run(
        &self,
        _context: &mut ZFContext,
        _state: &mut Box<dyn ZFStateTrait>,
        inputs: &mut HashMap<String, ZFDataMessage>,
    ) -> ZFResult<HashMap<zenoh_flow::ZFPortID, Arc<dyn zenoh_flow::ZFDataTrait>>> {
        let mut results = HashMap::<String, Arc<dyn ZFDataTrait>>::with_capacity(1);
        let (_, usize0) = get_input!(ZFU64, String::from(LINK_ID_TICK), inputs)?;
        if let Ok( value) = get_input!(ZFUsize, String::from(LINK_ID_INPUT1), inputs) {
            results.insert(
                String::from(LINK_ID_OUTPUT),
                zf_data!(ZFString(format!(
                    "{}+{}={:?}",
                    usize0.0,
                    value.1.0,
                    thread::current().id()
                ))),
            );
        }

        // let api = unsafe { std::pin::Pin::new_unchecked(&mut *API) };
        // api.SpinSome();
        
        Ok(results)
    }
}

impl ZFComponentOutputRule for OperatorSimplePlanningSimulator {
    fn output_rule(
        &self,
        _context: &mut ZFContext,
        state: &mut Box<dyn ZFStateTrait>,
        outputs: &HashMap<String, Arc<dyn ZFDataTrait>>,
    ) -> ZFResult<HashMap<zenoh_flow::ZFPortID, ZFComponentOutput>> {
        default_output_rule(state, outputs)
    }
}

export_operator!(register);

fn register() -> ZFResult<Arc<dyn ZFOperatorTrait>> {
    Ok(Arc::new(OperatorSimplePlanningSimulator) as Arc<dyn ZFOperatorTrait>)
}
