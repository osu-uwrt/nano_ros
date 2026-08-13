
use std::{fmt::format, path::PathBuf};
use std::io::{BufWriter, Write};

use crate::ros_toml_schema::*;
use crate::msg_file_parser::*;
use std::collections::HashSet;
use anyhow::{Result, anyhow};

const HEADER_FILE_NAME: &str = "nano_ros_codegen.h";
const CODE_FILE_NAME: &str = "nano_ros_serde_codegen.c";
const IMPLEMENTATION_FILE_NAME: &str = "nano_ros_impl_codegen.c";
const HEADER_INCLUDE_LINES: &[&str] = &[
    "#pragma once",
    "#include <stdbool.h>",
    "#include <stdint.h>",
    //"#include <ucdr/microcdr.h>",
    "#include \"nano_ros.h\"",
    ""
];
const CODE_INCLUDE: &str = "#include \"nano_ros_codegen.h\"\n";
const IMPL_INCLUDE: &str = "#include \"nano_ros_codegen.h\"\n";

pub fn create_hash() {

}

pub fn generate_serialize_function(out: &mut impl Write, field: &MsgField) -> Result<()> {
    match &field.field_type {
        FieldType::Nested(c_type, _) => {
            writeln!(out, "    ok &= {}__serialize(buf, &msg->{});", c_type.trim_end_matches("_t"), field.name)?;
        },
        FieldType::NestedArray(c_type, _, len) => {
            writeln!(out, "    for (int i = 0; i < {}; i++) {{", len)?;
            writeln!(out, "        ok &= {}__serialize(buf, &msg->{}[i]);", c_type.trim_end_matches("_t"), field.name)?;
            writeln!(out, "    }}")?;
        }, 
        FieldType::PrimArray(prim, len) => {
            writeln!(out, "    ok &= ucdr_serialize_array_{}(buf, msg->{}, {});", prim.c_type(), field.name, len)?;
        },
        FieldType::Primitive(prim) => {
            match prim {
                PrimType::String => {
                    writeln!(out, "    ok &= ucdr_serialize_string(buf, msg->{});", field.name)?;
                }, 
                _ => {
                    writeln!(out, "    ok &= ucdr_serialize_{}(buf, msg->{});", prim.c_type(), field.name)?;
                }
            }
        },
    }
    Ok(())
}

pub fn generate_deserialize_function(out: &mut impl Write, field: &MsgField) -> Result<()> {
    match &field.field_type {
        FieldType::Nested(c_type, _) => {
            writeln!(out, "    ok &= {}__deserialize(buf, &msg->{});", c_type.trim_end_matches("_t"), field.name)?;
        },
        FieldType::NestedArray(c_type, _, len) => {
            writeln!(out, "    for (int i = 0; i < {}; i++) {{", len)?;
            writeln!(out, "        ok &= {}__deserialize(buf, &msg->{}[i]);", c_type.trim_end_matches("_t"), field.name)?;
            writeln!(out, "    }}")?;
        },
        FieldType::PrimArray(prim, len) => {
            writeln!(out, "    ok &= ucdr_deserialize_array_{}(buf, msg->{}, {});", prim.c_type(), field.name, len)?;
        },
        FieldType::Primitive(prim) => {
            match prim {
                PrimType::String => {
                    writeln!(out, "    ok &= ucdr_deserialize_string(buf, msg->{}, sizeof(msg->{}));", field.name, field.name)?;
                }, 
                _ => {
                    writeln!(out, "    ok &= ucdr_deserialize_{}(buf, &msg->{});", prim.c_type(), field.name)?;
                }
            }
        },
    }
    Ok(()) 
}

pub fn generate_serde_functions(fields: &[MsgField], type_prefix: &str, out: &mut impl Write, emitted: &mut HashSet<String>, exposed_serde: &mut Vec<String>) -> Result<()> {
    if !emitted.insert(type_prefix.to_string()) {
        return Ok(())
    }
    for field in fields {
        match &field.field_type {
            FieldType::Nested(c_type, nested_fields) => {
                let nested_prefix = c_type.trim_end_matches("_t");
                generate_serde_functions(nested_fields, nested_prefix, out, emitted, exposed_serde)?;
            }
            FieldType::NestedArray(c_type, nested_fields, _) => {
                let nested_prefix = c_type.trim_end_matches("_t");
                generate_serde_functions(nested_fields, nested_prefix, out, emitted, exposed_serde)?;
            }
            _ => {}
        }
    }

    let serialize_fn = format!("bool {prefix}__serialize(ucdrBuffer* buf, const {prefix}_t* msg)", prefix = type_prefix);
    exposed_serde.push(format!("{serialize};", serialize = serialize_fn));

    writeln!(out, "{serialize} {{", serialize = serialize_fn)?;
    writeln!(out, "    bool ok = true;")?;
    for field in fields {
        generate_serialize_function(out, &field)?;
    }
    writeln!(out, "    return ok;\n}}\n")?;

    let deserialize_fn = format!("bool {prefix}__deserialize(ucdrBuffer* buf, {prefix}_t* msg)", prefix = type_prefix);
    exposed_serde.push(format!("{deserialize};", deserialize = deserialize_fn));

    writeln!(out, "{deserialize} {{", deserialize = deserialize_fn)?;
    writeln!(out, "    bool ok = true;")?;
    for field in fields {
        generate_deserialize_function(out, &field)?;
    }
    writeln!(out, "    return ok;\n}}\n")?;

    Ok(())
}

pub fn generate_user_c_struct(fields: &[MsgField], struct_name: &str, out: &mut impl Write, emitted: &mut HashSet<String>) -> Result<()> {
    if !emitted.insert(struct_name.to_string()) {
        return Ok(())
    }
    for field in fields {
        match &field.field_type {
            FieldType::Nested(c_type, nested_fields) => {
                generate_user_c_struct(nested_fields, c_type, out, emitted)?
            },
            FieldType::NestedArray(c_type,nested_fields , _) => {
                generate_user_c_struct(nested_fields, c_type, out, emitted)?
            },
            _ => {}
        }
    }

    writeln!(out, "typedef struct {{")?;
    for field in fields {
        match &field.field_type {
            FieldType::Primitive(prim) => {
                match prim {
                    PrimType::String => {
                        writeln!(out, "    {} {}[{}];", prim.c_type(), field.name, 256)?
                    },
                    _ => {
                        writeln!(out, "    {} {};", prim.c_type(), field.name)?
                    }
                }
            },
            FieldType::PrimArray(prim, len) => {
                writeln!(out, "    {} {}[{}];", prim.c_type(), field.name, len)?
            },
            FieldType::Nested(c_type, _) => {
                writeln!(out, "    {} {};", c_type, field.name)?
            },
            FieldType::NestedArray(c_type, _, len) => {
                writeln!(out, "    {} {}[{}]", c_type, field.name, len)?
            },
        }
    }
    writeln!(out, "}} {};\n", struct_name)?;

    Ok(())
}

/// Brief generates the user facing api prerequisites and lays out memory statically
/// 
/// for example:
pub fn generate_user_function(topic: &Topic, code_out: &mut impl Write, user_funcs: &mut Vec<String>) -> Result<()> {
    let struct_name = ros_type_to_c_struct_name(&topic.ros_type);
    let prefix = struct_name.trim_end_matches("_t");
    let serialize_fn = format!("(nros_serialize_fn) {}__serialize", prefix);

    match topic.direction {
        Direction::Publisher => {
            writeln!(code_out, "bool nros_publish_{}(const {} *msg) {{", topic.name, struct_name)?;
            writeln!(code_out, "    return nros_publish({}_ID, {}, msg);", topic.name.to_uppercase(), serialize_fn)?;
            writeln!(code_out, "}}\n")?;   
            user_funcs.push(format!("bool nros_publish_{}(const {} *msg);", topic.name, struct_name));
        },
        Direction::Subscriber => {
            writeln!(code_out, "static void (*{}_cb)(const {} *) = NULL;\n", topic.name, struct_name)?;

            writeln!(code_out, "static void nros_{}_subscriber_handler(ucdrBuffer *buf) {{", topic.name)?;
            writeln!(code_out, "    {} msg;", struct_name)?;
            writeln!(code_out, "    if ({}__deserialize(buf, &msg) && {}_cb) {{", prefix, topic.name)?;
            writeln!(code_out, "        {}_cb(&msg);", topic.name)?;
            writeln!(code_out, "    }}")?;
            writeln!(code_out, "}}\n")?;

            writeln!(code_out, "void nros_set_{}_subscriber_cb(void (*cb)(const {} *msg)) {{", topic.name, struct_name)?;
            writeln!(code_out, "    {}_cb = cb;", topic.name)?;
            writeln!(code_out, "}}\n")?;
            user_funcs.push(format!("void nros_set_{}_subscriber_cb(void (*cb)(const {} *msg));", topic.name, struct_name));
        }
    }
    Ok(())
}

pub fn generate_topic_ids(topics: &Vec<Topic>, out: &mut impl Write) -> Result<()> {
    let mut sub_id: u8 = 0;
    let mut pub_id: u8 = 127;
    for topic in topics {
        match topic.direction {
            Direction::Publisher => {
                if pub_id >= 255 {
                    return Err(anyhow!("publisher {} id out of range", topic.name))
                } else {
                    writeln!(out, "#define {}_ID {}u", topic.name.to_uppercase(), pub_id)?;
                    pub_id += 1;
                }
            },
            Direction::Subscriber => {
                if sub_id >= 127 {
                    return Err(anyhow!("subscriber {} id out of range", topic.name));
                } else {
                    writeln!(out, "#define {}_ID {}u", topic.name.to_uppercase(), sub_id)?;
                    sub_id += 1;
                }
            }
        }
    }
    writeln!(out)?;
    Ok(())
}

pub fn generate_topics(topics: &Vec<Topic>, out: &mut impl Write) -> Result<()> {

    Ok(())
}
// could gen a generic publish function for publishers which is assigned at gen time: publisher->publish(data)
pub fn generate_topic_struct_decl(out: &mut impl Write) -> Result<()> {
    writeln!(out, "typedef struct {{")?;
    writeln!(out, "    uint8_t topic_id;")?; // 0 - 127 reserved for subs, 128 - 255 reserved for pubs
    writeln!(out, "    const char *topic_name;")?;
    writeln!(out, "    const char *topic_type;")?;
    writeln!(out, "    void (*cb)(ucdrBuffer *buf);")?; // used for subs
    writeln!(out, "}} nros_topic_t;\n")?;
    Ok(())
}

pub fn generate_topic_array(topics: &Vec<Topic>, out: &mut impl Write) -> Result<()> {
    writeln!(out, "const uint8_t nros_topic_count = {}u;", topics.len())?;
    writeln!(out, "const nros_topic_t nros_topics[{}] = {{", topics.len())?;
    for topic in topics {
        writeln!(out, "    {{")?;
        writeln!(out, "        .topic_id = {}_ID,", topic.name.to_uppercase())?;
        writeln!(out, "        .topic_name = \"{}\",", topic.name)?;
        writeln!(out, "        .topic_type = \"{}\",", topic.ros_type.replace("/", "/msg/"))?;
        match topic.direction {
            Direction::Publisher => {
                writeln!(out, "        .cb = NULL,")?;
            },
            Direction::Subscriber => {
                writeln!(out, "        .cb = nros_{}_subscriber_handler,", topic.name)?;
            },
        }
        //writeln!(out, "        .qos = ")?;
        writeln!(out, "    }},")?;
    }
    writeln!(out, "}};\n")?;
    Ok(())
}

pub fn generate_lib_externs(out: &mut impl Write) -> Result<()> {
    Ok(())
}

pub fn generate_function_protos(user_funcs: &Vec<String>, exposed_serde: &Vec<String>, out: &mut impl Write) -> Result<()> {
    for func in user_funcs {
        writeln!(out, "{}", func)?;
    }
    for serde in exposed_serde {
        writeln!(out, "{}", serde)?;
    }
    Ok(())
}

pub fn generate_core_code(config: &NanoRosConfig, out_dir: &PathBuf) -> Result<(HashSet<String>, HashSet<String>)> {
    std::fs::DirBuilder::new().recursive(true).create(out_dir)?;
    let mut header = BufWriter::new(std::fs::File::create(out_dir.join(HEADER_FILE_NAME))?);
    let mut code = BufWriter::new(std::fs::File::create(out_dir.join(CODE_FILE_NAME))?);
    let mut impl_code = BufWriter::new(std::fs::File::create(out_dir.join(IMPLEMENTATION_FILE_NAME))?);

    for line in HEADER_INCLUDE_LINES {
        writeln!(header, "{}", line)?;
    }
    writeln!(header)?;

    generate_topic_ids(&config.topics, &mut header)?;
    //generate_topic_struct_decl(&mut header)?;
    generate_lib_externs(&mut header)?;

    writeln!(code, "{}", CODE_INCLUDE)?;

    writeln!(impl_code, "{}", IMPL_INCLUDE)?;

    let mut emitted_structs: HashSet<String> = HashSet::new();
    let mut emitted_functions: HashSet<String> = HashSet::new();
    let mut user_functions: Vec<String> = Vec::new();
    let mut exposed_serde: Vec<String> = Vec::new();
    for topic in &config.topics {
        let struct_name = ros_type_to_c_struct_name(&topic.ros_type);
        let function_prefix = ros_type_to_c_prefix(&topic.ros_type);
        //generate_topic_ids();
        generate_user_c_struct(&topic.fields, &struct_name, &mut header, &mut emitted_structs)?;
        generate_serde_functions(&topic.fields, &function_prefix, &mut code, &mut emitted_functions, &mut exposed_serde)?;
        generate_user_function(topic, &mut impl_code, &mut user_functions)?;
    }
    generate_topic_array(&config.topics, &mut impl_code)?;
    generate_function_protos(&user_functions, &exposed_serde, &mut header)?; 

    Ok((emitted_structs, emitted_functions))
}