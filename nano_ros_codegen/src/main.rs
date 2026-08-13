use std::env::args;
use std::path::PathBuf;
use anyhow::{Result, anyhow, bail};

use crate::ros_toml_schema::NanoRosConfig;
mod ros_toml_schema;
mod msg_file_parser;
mod codegen;

use msg_file_parser::*;
use codegen::*; 

fn parse_args(args: &[String]) -> Result<(PathBuf, PathBuf, PathBuf, PathBuf)> {
    let mut toml_path: Option<PathBuf> = None;
    let mut ros_prefix: Option<PathBuf> = None;
    let mut user_types_path: Option<PathBuf> = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--ros-prefix" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| anyhow!("--ros-prefix requires a value"))?;
                ros_prefix = Some(PathBuf::from(val));
            }
            "--user-msgs" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| anyhow!("--user-msgs requires a value"))?;
                user_types_path = Some(PathBuf::from(val));
            }
            arg if !arg.starts_with("--") => {
                toml_path = Some(PathBuf::from(arg));
            }
            other => {
                bail!("Unknown argument: {other}");
            }
        }
        i += 1;
    }

    let toml_path = toml_path.ok_or_else(|| anyhow!("must provide path to ros.toml"))?;

    let out_dir = toml_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join("generated");

    let ros_prefix = match ros_prefix {
        Some(p) => p,
        None => match std::env::var("AMENT_PREFIX_PATH") {
            Ok(env_val) => PathBuf::from(env_val.split(':').next().unwrap_or("/opt/ros/humble")),
            Err(_) => {
                eprintln!(
                    "[ros_codegen] WARNING: --ros-prefix not set and AMENT_PREFIX_PATH not found. Defaulting to /opt/ros/humble"
                );
                PathBuf::from("/opt/ros/humble")
            }
        },
    };

    let user_types_path = user_types_path.unwrap_or_else(|| PathBuf::from(""));

    Ok((toml_path, ros_prefix, user_types_path, out_dir))
}

fn main() -> Result<()> {
    let args: Vec<String> = args().collect();
    let (toml_path, ros_prefix, user_types, out_dir) = parse_args(&args)?;
    println!("toml_path: {}, ros_pref: {}, user_types_dir: {}, out_dir: {}", toml_path.display(), ros_prefix.display(), user_types.display(), out_dir.display());

    let toml_str = std::fs::read_to_string(toml_path)?;
    let mut config: NanoRosConfig = toml::from_str(&toml_str)?;

    let board_name = &config.board.name;

    println!("[nano_ros_codegen] Board: {board_name}, Transport: {:?}", &config.board.transport);

    for topic in &mut config.topics {
        //println!("\nsearching for [{:?}]\n", topic.ros_type);
        let path = find_ros_msg_file(&ros_prefix, &topic.ros_type, &user_types)?;
        //println!("path to type: {}", path.display());
        let fields = parse_msg_file(&ros_prefix, &user_types, &path)?;
        // for field in &fields {
        //     println!("FIELD NAME: {}", field.name);
        //     match &field.field_type {
        //         FieldType::Primitive(prim) => { println!("PRIMITIVE FIELD...")},
        //         FieldType::PrimArray(prim, len) => { println!("PRIMITIVE ARRAY OF SIZE: {}", len)},
        //         FieldType::Nested(type_name, nested_fields ) => {println!("NESTED TYPE: {}", type_name)},
        //         FieldType::NestedArray(type_name, nested_fields , len) => { println!("ARRAY OF NESTED TYPE: {}", type_name)}
        //     }
        // }
        topic.fields = fields;


        //all_msg_fields.push(fields);

    }
    //generate_core_code(&config, &out_dir)?;
    let (structs, functions) = generate_core_code(&config, &out_dir)?;

    Ok(())
}
