use zenoh_flow::serde::{Deserialize, Serialize};
use zenoh_flow::zenoh_flow_derive::{ZFData, ZFState};
use zenoh_flow::{ZFDataTrait, ZFResult};

#[derive(Debug, Clone, Serialize, Deserialize, ZFData)]
pub struct ZFString(pub String);

impl From<String> for ZFString {
    fn from(s: String) -> Self {
        ZFString(s)
    }
}

impl From<&str> for ZFString {
    fn from(s: &str) -> Self {
        ZFString(s.to_owned())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, ZFData)]
pub struct ZFUsize(pub usize);

impl ZFDataTrait for ZFUsize {
    fn try_serialize(&self) -> ZFResult<Vec<u8>> {
        Ok(self.0.to_ne_bytes().to_vec())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, ZFState)]
pub struct ZFEmptyState;

#[derive(Serialize, Deserialize, Debug, Clone, ZFData)]
pub struct RandomData {
    pub d: u64,
}

#[derive(Serialize, Deserialize, Debug, Clone, ZFData)]
pub struct ZFBytes {
    pub bytes: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize, ZFData)]
pub struct ZFDouble(pub f64);
