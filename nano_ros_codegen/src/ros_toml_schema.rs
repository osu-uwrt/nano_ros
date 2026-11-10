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

    /// ros.schema.json is written by hand for editors; this keeps its enum values in sync with
    /// what codegen accepts.
    #[test]
    fn json_schema_enums_match() {
        let schema: serde_json::Value = serde_json::from_str(include_str!("../ros.schema.json")).unwrap();
        let topic = &schema["properties"]["topics"]["items"]["properties"];
        let strings = |v: &serde_json::Value| -> Vec<String> {
            v.as_array().unwrap().iter().map(|x| x.as_str().unwrap().to_string()).collect()
        };

        let qos: Vec<String> = topic["qos"]["oneOf"].as_array().unwrap().iter()
            .map(|v| v["const"].as_str().unwrap().to_string()).collect();
        for q in &qos {
            assert!(parse(&format!("qos = \"{q}\"")).is_ok(), "schema lists qos {q}, codegen rejects it");
        }
        assert_eq!(qos.len(), 6, "a Qos variant was added or removed: update ros.schema.json");

        for t in strings(&topic["topic_type"]["enum"]) {
            let toml = format!("[board]\nname = \"b\"\ntransport = \"can\"\n\n[[topics]]\nname = \"t\"\ntopic_type = \"{t}\"\nmsg_type = \"std_msgs/msg/UInt8\"\n");
            assert!(toml::from_str::<NanoRosConfig>(&toml).is_ok(), "schema lists topic_type {t}, codegen rejects it");
        }
        for t in strings(&schema["properties"]["board"]["properties"]["transport"]["enum"]) {
            let toml = format!("topics = []\n\n[board]\nname = \"b\"\ntransport = \"{t}\"\n");
            assert!(toml::from_str::<NanoRosConfig>(&toml).is_ok(), "schema lists transport {t}, codegen rejects it");
        }
    }
}
