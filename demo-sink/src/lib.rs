use async_trait::async_trait;
use std::collections::HashMap;
use zenoh_flow::runtime::message::ZFDataMessage;
use zenoh_flow::{
    default_input_rule, export_sink, types::ZFResult, zf_empty_state, Token, ZFComponent,
    ZFComponentInputRule, ZFStateTrait,
};
use zenoh_flow::{ZFContext, ZFSinkTrait};

struct GenericSink;

#[async_trait]
impl ZFSinkTrait for GenericSink {
    async fn run(
        &self,
        _context: &mut ZFContext,
        _state: &mut Box<dyn ZFStateTrait>,
        inputs: &mut HashMap<String, ZFDataMessage>,
    ) -> ZFResult<()> {
        for (k, v) in inputs {
            println!("Demo Sink Received on LinkId {:?} -> {:?}", k, v);
        }
        Ok(())
    }
}

impl ZFComponent for GenericSink {
    fn initial_state(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn ZFStateTrait> {
        zf_empty_state!()
    }
}

impl ZFComponentInputRule for GenericSink {
    fn input_rule(
        &self,
        _context: &mut ZFContext,
        state: &mut Box<dyn ZFStateTrait>,
        tokens: &mut HashMap<String, Token>,
    ) -> ZFResult<bool> {
        default_input_rule(state, tokens)
    }
}

export_sink!(register);

fn register() -> ZFResult<Box<dyn ZFSinkTrait + Send>> {
    Ok(Box::new(GenericSink) as Box<dyn ZFSinkTrait + Send>)
}
