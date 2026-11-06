use serde::Deserialize;
use std::collections::BTreeMap;
use toml::Spanned;
use crate::msg_file_parser::MsgField;

#[derive(Debug, Deserialize, PartialEq, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum Transport {
    Usb,
    Can,
}

#[derive(Debug, Deserialize, PartialEq, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum TopicType {
    Publisher,
    Subscriber,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardInfo {
    pub name: String,
    pub transport: Transport,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Topic {
    pub name: Spanned<String>,
    pub topic_type: TopicType,
    pub msg_type: Spanned<String>,
    /// Caps for variable-length arrays, by field path: { data = 64, "layout.dim" = 4 }
    #[serde(default)]
    pub max_len: BTreeMap<Spanned<String>, Spanned<usize>>,
    #[serde(skip)]
    pub fields: Vec<MsgField>,  // populated after parsing .msg file
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NanoRosConfig {
    pub board: BoardInfo,
    pub topics: Vec<Topic>,
}