use serde::Deserialize;
use std::path::PathBuf;
use crate::msg_file_parser::MsgField;

#[derive(Debug, Deserialize, PartialEq, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum Transport {
    Usb,
    Can,
}

#[derive(Debug, Deserialize, PartialEq, Clone, Copy)]
#[serde(rename_all = "lowercase")]
pub enum Direction {
    Publisher,
    Subscriber,
}

#[derive(Debug, Deserialize)]
pub struct BoardInfo {
    pub name: String,
    pub transport: Transport,
}

#[derive(Debug, Deserialize)]
pub struct Topic {
    pub name: String,
    pub direction: Direction,
    pub ros_type: String,
    pub qos: String,
    #[serde(skip)]
    pub fields: Vec<MsgField>,  // populated after parsing .msg file
}

#[derive(Debug, Deserialize)]
pub struct NanoRosConfig {
    pub board: BoardInfo,
    pub topics: Vec<Topic>,
}