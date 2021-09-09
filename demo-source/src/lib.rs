use async_std::sync::Arc;
use async_trait::async_trait;
use demo_types::ZFUsize;
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use zenoh_flow::{
    default_output_rule, export_source, zf_data, zf_empty_state, ZFComponent,
    ZFComponentOutputRule, ZFContext, ZFDataTrait, ZFPortID, ZFResult, ZFSourceTrait, ZFStateTrait,
};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

#[derive(Debug)]
struct DemoSource;

impl ZFComponent for DemoSource {
    fn initialize(
        &self,
        _configuration: &Option<HashMap<String, String>>,
    ) -> Box<dyn zenoh_flow::ZFStateTrait> {
        zf_empty_state!()
    }
    fn clean(&self, _state: &mut Box<dyn ZFStateTrait>) -> ZFResult<()> {
        Ok(())
    }
}

impl ZFComponentOutputRule for DemoSource {
    fn output_rule(
        &self,
        _context: &mut ZFContext,
        state: &mut Box<dyn zenoh_flow::ZFStateTrait>,
        outputs: &HashMap<String, Arc<dyn zenoh_flow::ZFDataTrait>>,
    ) -> ZFResult<HashMap<ZFPortID, zenoh_flow::ZFComponentOutput>> {
        default_output_rule(state, outputs)
    }
}

#[async_trait]
impl ZFSourceTrait for DemoSource {
    async fn run(
        &self,
        _context: &mut ZFContext,
        _state: &mut Box<dyn zenoh_flow::ZFStateTrait>,
    ) -> ZFResult<HashMap<ZFPortID, Arc<dyn ZFDataTrait>>> {
        let mut results: HashMap<ZFPortID, Arc<dyn ZFDataTrait>> = HashMap::with_capacity(1);
        results.insert(
            String::from("input"),
            zf_data!(ZFUsize(COUNTER.fetch_add(1, Ordering::AcqRel))),
        );
        async_std::task::sleep(std::time::Duration::from_millis(100)).await;
        Ok(results)
    }
}

// Also generated by macro
export_source!(register);

fn register() -> ZFResult<Arc<dyn ZFSourceTrait>> {
    Ok(Arc::new(DemoSource) as Arc<dyn ZFSourceTrait>)
}
