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
pub enum TopicType {
    Publisher,
    Subscriber,
}

/// QoS profile for a topic. Each variant maps to a `nros_qos_t` value in protocol.h.
#[derive(Debug, Deserialize, PartialEq, Clone, Copy, Default)]
#[serde(rename_all = "snake_case")]
pub enum Qos {
    #[default]
    Default,
    SensorData,
    Reliable,
    BestEffort,
    TransientLocal,
    SystemDefault,
}

impl Qos {
    /// Name of the matching `nros_qos_t` constant in protocol.h.
    pub fn c_name(&self) -> &'static str {
        match self {
            Qos::Default => "NROS_QOS_DEFAULT",
            Qos::SensorData => "NROS_QOS_SENSOR_DATA",
            Qos::Reliable => "NROS_QOS_RELIABLE",
            Qos::BestEffort => "NROS_QOS_BEST_EFFORT",
            Qos::TransientLocal => "NROS_QOS_TRANSIENT_LOCAL",
            Qos::SystemDefault => "NROS_QOS_SYSTEM_DEFAULT",
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct BoardInfo {
    pub name: String,
    pub transport: Transport,
}

#[derive(Debug, Deserialize)]
pub struct Topic {
    pub name: String,
    pub topic_type: TopicType,
    pub msg_type: String,
    #[serde(default)]
    pub qos: Qos,
    #[serde(skip)]
    pub fields: Vec<MsgField>,  // populated after parsing .msg file
}

#[derive(Debug, Deserialize)]
pub struct NanoRosConfig {
    pub board: BoardInfo,
    pub topics: Vec<Topic>,
}
#[cfg(test)]
mod tests {
    use super::*;

    fn parse(qos_line: &str) -> Result<NanoRosConfig, toml::de::Error> {
        toml::from_str(&format!(
            "[board]\nname = \"b\"\ntransport = \"can\"\n\n[[topics]]\nname = \"t\"\ntopic_type = \"publisher\"\nmsg_type = \"std_msgs/msg/UInt8\"\n{qos_line}\n"
        ))
    }

    #[test]
    fn qos_defaults_when_omitted() {
        assert_eq!(parse("").unwrap().topics[0].qos, Qos::Default);
    }

    #[test]
    fn qos_parses_snake_case() {
        assert_eq!(parse("qos = \"sensor_data\"").unwrap().topics[0].qos, Qos::SensorData);
        assert_eq!(parse("qos = \"transient_local\"").unwrap().topics[0].qos, Qos::TransientLocal);
    }

    #[test]
    fn qos_rejects_unknown() {
        assert!(parse("qos = \"bogus\"").is_err());
    }
}
