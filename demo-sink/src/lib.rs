use async_std::sync::Arc;
use async_trait::async_trait;
use std::collections::HashMap;
use zenoh_flow::runtime::message::ZFDataMessage;
use zenoh_flow::{
    default_input_rule, export_sink, types::ZFResult, zf_empty_state, Token, ZFComponent,
    ZFComponentInputRule, ZFContext, ZFSinkTrait, ZFStateTrait,
};

struct DemoSink;

#[async_trait]
impl ZFSinkTrait for DemoSink {
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

impl ZFComponent for DemoSink {
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn ZFStateTrait> {
        zf_empty_state!()
    }
    fn clean(&self, _state: &mut Box<dyn ZFStateTrait>) -> ZFResult<()> {
        Ok(())
    }
}

impl ZFComponentInputRule for DemoSink {
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

fn register() -> ZFResult<Arc<dyn ZFSinkTrait>> {
    Ok(Arc::new(DemoSink) as Arc<dyn ZFSinkTrait>)
}
