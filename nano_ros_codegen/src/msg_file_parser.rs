use std::path::{Path, PathBuf};
use anyhow::{Context, Result, anyhow};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PrimType {
    Bool,
    Byte,
    Char,
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    Uint8,
    Uint16,
    Uint32,
    Uint64,
    String, // bounded/unbounded — treated as char[] on firmware (warn)
}

#[derive(Debug, Clone)]
pub enum FieldType {
    /// Primitive scalar: bool, uint8, int32, float64, etc.
    Primitive(PrimType),
    /// Fixed-length array of a primitive: float64[3]
    PrimArray(PrimType, usize),
    /// Nested message type, fully resolved to its fields.
    Nested(String, Vec<MsgField>), // (C type name, fields)
    /// Fixed-length array of nested type
    NestedArray(String, Vec<MsgField>, usize),
}

/// A single field in a .msg definition, after resolution.
#[derive(Debug, Clone)]
pub struct MsgField {
    pub name: String,
    pub field_type: FieldType,
}

impl PrimType {
    pub fn c_type(&self) -> &'static str {
        match self {
            PrimType::Bool => "bool",
            PrimType::Byte | PrimType::Uint8 | PrimType::Char => "uint8_t",
            PrimType::Float32 => "float",
            PrimType::Float64 => "double",
            PrimType::Int8 => "int8_t",
            PrimType::Int16 => "int16_t",
            PrimType::Int32 => "int32_t",
            PrimType::Int64 => "int64_t",
            PrimType::Uint16 => "uint16_t",
            PrimType::Uint32 => "uint32_t",
            PrimType::Uint64 => "uint64_t",
            PrimType::String => "char",
        }
    }

    /// Alignment requirement in bytes (CDR natural alignment)
    pub fn align(&self) -> usize {
        match self {
            PrimType::Bool | PrimType::Byte | PrimType::Char | PrimType::Int8 | PrimType::Uint8 => 1,
            PrimType::Int16 | PrimType::Uint16 => 2,
            PrimType::Float32 | PrimType::Int32 | PrimType::Uint32 => 4,
            PrimType::Float64 | PrimType::Int64 | PrimType::Uint64 => 8,
            PrimType::String => 4, // length prefix is u32
        }
    }

    /// size in bytes
    pub fn size(&self) -> usize {
        match self {
            PrimType::Bool | PrimType::Byte | PrimType::Char | PrimType::Int8 | PrimType::Uint8 => 1,
            PrimType::Int16 | PrimType::Uint16 => 2,
            PrimType::Float32 | PrimType::Int32 | PrimType::Uint32 => 4,
            PrimType::Float64 | PrimType::Int64 | PrimType::Uint64 => 8,
            PrimType::String => 0,
        }
    }

}

fn parse_primitive(s: &str) -> Option<PrimType> {
    match s {
        "bool" => Some(PrimType::Bool),
        "byte" => Some(PrimType::Byte),
        "char" => Some(PrimType::Char),
        "float32" => Some(PrimType::Float32),
        "float64" => Some(PrimType::Float64),
        "int8" => Some(PrimType::Int8),
        "int16" => Some(PrimType::Int16),
        "int32" => Some(PrimType::Int32),
        "int64" => Some(PrimType::Int64),
        "uint8" => Some(PrimType::Uint8),
        "uint16" => Some(PrimType::Uint16),
        "uint32" => Some(PrimType::Uint32),
        "uint64" => Some(PrimType::Uint64),
        "string" | "wstring" => Some(PrimType::String),
        _ => None,
    }
}

pub fn find_ros_msg_file(ros_prefix: &Path, type_path: &str, user_types_path: &Path) -> Result<PathBuf> {
    let parts: Vec<&str> = type_path.split("/").collect();
    // if parts.len() != 3 {
    //     return Err(anyhow!("Invalid message type: {}, expected 'pkg/msg/type'", type_path));
    // }

    //let (pkg, _subdir, name) = (parts[0], parts[1], parts[2]);
    let (pkg, name) = (parts[0], parts[1]);
    let path_to_ros = ros_prefix.join("share").join(pkg).join("msg").join(format!("{}.msg", name));
    let path_to_user = user_types_path.join(pkg).join("msg").join(format!("{}.msg", name));
    if path_to_ros.exists() {
        return Ok(path_to_ros);
    } else if path_to_user.exists() {
        return Ok(path_to_user);
    } else {
        return Err(anyhow!("msg file for: {} not found in user types directory or ros installation", type_path));
    }

    // Ok(path)
}

pub fn parse_msg_file(ros_prefix: &Path, user_types_path: &Path, msg_path: &Path) -> Result<Vec<MsgField>> {
    let content = std::fs::read_to_string(msg_path)
        .with_context(|| format!("Failed to read {}", msg_path.display()))?;
    parse_msg_str(ros_prefix, user_types_path, &content, msg_path)
}

fn parse_msg_str(ros_prefix: &Path, user_types_path: &Path, content: &str, source: &Path) -> Result<Vec<MsgField>> {
    let mut fields = Vec::new();
    for line in content.lines() {
        let line = match line.split('#').next() {
            Some(l) => l.trim(),
            None => continue,
        };
        if line.is_empty() {
            continue;
        }

        // Constants: "TYPE NAME=VALUE" — skip
        if line.contains('=') {
            continue;
        }

        // Parse: TYPE[N] NAME  or  TYPE NAME
        let mut parts = line.split_whitespace();
        let type_token = match parts.next() {
            Some(t) => t,
            None => continue,
        };
        let field_name = match parts.next() {
            Some(n) => n.to_string(),
            None => continue,
        };

        let (base_type_str, array_len) = if let Some(bracket) = type_token.find("[") {
            let base = &type_token[..bracket];
            let rest = &type_token[bracket + 1..];
            let len_str = rest.trim_end_matches(']');
            let len = if len_str.is_empty() {
                None
            } else {
                Some(len_str.parse::<usize>().with_context(|| {
                    format!("Invalid array length in '{}' at {}", type_token, source.display())
                })?)
            };
            (base, len)
        } else {
            (type_token, None)
        };

        if let Some(prim) = parse_primitive(base_type_str) {
            match array_len {
                Some(n) => fields.push(MsgField {
                    name: field_name,
                    field_type: FieldType::PrimArray(prim, n)
                }),
                None => fields.push(MsgField {
                    name: field_name,
                    field_type: FieldType::Primitive(prim)
                }),
            }

        } else {
            let nested_type_str = resolve_nested_type(base_type_str, source);
            let nested_path = find_ros_msg_file(ros_prefix, &nested_type_str, user_types_path).with_context(|| format!("Resolving nested type '{}' for field '{}'", base_type_str, field_name))?;
            let nested_fields = parse_msg_file(ros_prefix, user_types_path, &nested_path)?;
            let c_type_name = ros_type_to_c_struct_name(&nested_type_str);

            match array_len {
                Some(n) => fields.push(MsgField { 
                    name: field_name, 
                    field_type: FieldType::NestedArray(c_type_name, nested_fields, n)
                }),
                None => {
                    if array_len.is_none() && type_token.contains("[]") {
                        eprintln!(
                            "WARNING: field '{}' is a dynamic array — skipping (not supported for fixed-size CDR)",
                            field_name
                        );
                        continue;
                    }
                    fields.push(MsgField { 
                        name: field_name, 
                        field_type: FieldType::Nested(c_type_name, nested_fields)
                    });
                }
            }
        }
    }

    Ok(fields)
}

/// If a .msg file references a type like "Vector3" with no package prefix,
/// infer it belongs to the same package as the current file.
fn resolve_nested_type(type_str: &str, source: &Path) -> String {
    if type_str.contains('/') {
        return type_str.to_string();
    }
    // Infer package from path: .../share/PKG/msg/Foo.msg
    let components: Vec<_> = source.components().collect();
    // Find "share" component and take the next one as pkg
    for (i, c) in components.iter().enumerate() {
        if c.as_os_str() == "share" {
            if let Some(pkg) = components.get(i + 1) {
                return format!("{}/msg/{}", pkg.as_os_str().to_string_lossy(), type_str);
            }
        }
    }
    // Fallback: geometry_msgs is the most common source of unqualified types
    format!("geometry_msgs/msg/{}", type_str)
}

/// "sensor_msgs/Imu" -> "sensor_msgs__msg__Imu_t"  (rosidl_generator_c convention)
pub fn ros_type_to_c_struct_name(type_str: &str) -> String {
    let replaced = type_str.replace('/', "__");

    format!("{}_t", replaced)
}
 
/// "sensor_msgs/msg/Imu" -> "sensor_msgs__msg__Imu" (prefix for functions)
pub fn ros_type_to_c_prefix(type_str: &str) -> String {
    type_str.replace('/', "__")
}