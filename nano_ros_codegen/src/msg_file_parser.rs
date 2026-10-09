use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use anyhow::{Context, Result, anyhow};

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Package index: package name -> directory containing its package.xml
// ---------------------------------------------------------------------------

pub type PackageIndex = HashMap<String, PathBuf>;

/// Directory names never worth descending into when looking for packages.
fn is_skipped_dir(name: &str) -> bool {
    name.starts_with('.') || matches!(name, "build" | "install" | "log")
}

/// Walk `root`, and record every directory that contains a package.xml.
/// Once a package is found we do not descend into it (packages don't nest).
pub fn build_package_index(root: &Path) -> Result<PackageIndex> {
    if !root.is_dir() {
        return Err(anyhow!("package search root is not a directory: {}", root.display()));
    }
    let mut index = PackageIndex::new();
    let mut visited = HashSet::new();
    scan_dir(root, &mut index, &mut visited)?;
    Ok(index)
}

fn scan_dir(dir: &Path, index: &mut PackageIndex, visited: &mut HashSet<PathBuf>) -> Result<()> {
    // Canonicalize so symlink cycles / duplicate links are only visited once.
    let canon = match std::fs::canonicalize(dir) {
        Ok(c) => c,
        Err(_) => return Ok(()), // dangling symlink etc.
    };
    if !visited.insert(canon) {
        return Ok(());
    }

    let package_xml = dir.join("package.xml");
    if package_xml.is_file() {
        let name = parse_package_name(&package_xml)?;
        match index.get(&name) {
            Some(existing) => eprintln!(
                "WARNING: package '{}' found at both {} and {} — keeping the first",
                name,
                existing.display(),
                dir.display()
            ),
            None => {
                index.insert(name, dir.to_path_buf());
            }
        }
        return Ok(()); // don't recurse into a package
    }

    let mut subdirs: Vec<PathBuf> = std::fs::read_dir(dir)
        .with_context(|| format!("Failed to read directory {}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.is_dir()) // follows symlinks
        .collect();
    // read_dir order is unspecified; sort so collision handling is deterministic.
    subdirs.sort();

    for sub in subdirs {
        let skip = sub
            .file_name()
            .and_then(|n| n.to_str())
            .map(is_skipped_dir)
            .unwrap_or(false);
        if skip {
            continue;
        }
        scan_dir(&sub, index, visited)?;
    }
    Ok(())
}

/// Extract the <name> element from a package.xml.
/// Dependency-free: strips XML comments, then does a substring scan.
/// Swap in `roxmltree` here if you need to handle exotic files.
fn parse_package_name(package_xml: &Path) -> Result<String> {
    let raw = std::fs::read_to_string(package_xml)
        .with_context(|| format!("Failed to read {}", package_xml.display()))?;
    let content = strip_xml_comments(&raw);

    let start = content
        .find("<name>")
        .ok_or_else(|| anyhow!("no <name> element in {}", package_xml.display()))?
        + "<name>".len();
    let end = content[start..]
        .find("</name>")
        .ok_or_else(|| anyhow!("unterminated <name> element in {}", package_xml.display()))?
        + start;

    let name = content[start..end].trim();
    if name.is_empty() {
        return Err(anyhow!("empty <name> element in {}", package_xml.display()));
    }
    Ok(name.to_string())
}

fn strip_xml_comments(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(start) = rest.find("<!--") {
        out.push_str(&rest[..start]);
        match rest[start..].find("-->") {
            Some(end) => rest = &rest[start + end + 3..],
            None => return out, // unterminated comment: drop the remainder
        }
    }
    out.push_str(rest);
    out
}

// ---------------------------------------------------------------------------
// Resolver
// ---------------------------------------------------------------------------

pub struct MsgResolver {
    /// Built from opt/ros/<distro>/share
    ros_pkgs: PackageIndex,
    /// Built from the user types directory
    user_pkgs: PackageIndex,
}

/// Result of resolving "pkg/Name" to an actual file.
pub struct ResolvedMsg {
    pub path: PathBuf,
    pub pkg: String,
}

impl MsgResolver {
    /// `ros_share` is e.g. `/opt/ros/humble/share` (no `.join("share")` anymore).
    pub fn new(ros_share: &Path, user_types_path: &Path) -> Result<Self> {
        Ok(Self {
            ros_pkgs: build_package_index(ros_share)
                .with_context(|| format!("Indexing ROS packages in {}", ros_share.display()))?,
            user_pkgs: build_package_index(user_types_path)
                .with_context(|| format!("Indexing user packages in {}", user_types_path.display()))?,
        })
    }

    /// Resolve "pkg/Name" or "pkg/msg/Name" to a .msg file. ROS install wins over user types,
    /// but if the ROS package exists and lacks the message we still fall back to user types.
    pub fn find_msg_file(&self, type_path: &str) -> Result<ResolvedMsg> {
        let (pkg, name) = split_type_path(type_path)?;

        let candidates = [self.ros_pkgs.get(pkg), self.user_pkgs.get(pkg)];
        if candidates.iter().all(|c| c.is_none()) {
            return Err(anyhow!(
                "package '{}' (from '{}') not found in ROS install or user types directory",
                pkg, type_path
            ));
        }

        for pkg_dir in candidates.into_iter().flatten() {
            // Fast path: conventional layout
            let conventional = pkg_dir.join("msg").join(format!("{}.msg", name));
            if conventional.is_file() {
                return Ok(ResolvedMsg { path: conventional, pkg: pkg.to_string() });
            }
            // Fallback: bounded search scoped to just this package's directory
            if let Some(found) = search_for_msg(pkg_dir, name, 4) {
                return Ok(ResolvedMsg { path: found, pkg: pkg.to_string() });
            }
        }

        Err(anyhow!("msg '{}' not found in package '{}'", name, pkg))
    }

    /// Convenience entry point: "riptide_msgs2/FirmwareStatus" -> parsed fields.
    pub fn parse_type(&self, type_path: &str) -> Result<Vec<MsgField>> {
        let resolved = self.find_msg_file(type_path)?;
        parse_msg_file(self, &resolved.path, &resolved.pkg)
    }
}

/// Accepts "pkg/Name" or "pkg/msg/Name".
fn split_type_path(type_path: &str) -> Result<(&str, &str)> {
    let parts: Vec<&str> = type_path.split('/').collect();
    match parts.as_slice() {
        [pkg, name] => Ok((pkg, name)),
        [pkg, "msg", name] => Ok((pkg, name)),
        _ => Err(anyhow!(
            "Invalid message type '{}', expected 'pkg/Name' or 'pkg/msg/Name'",
            type_path
        )),
    }
}

fn search_for_msg(dir: &Path, name: &str, depth: usize) -> Option<PathBuf> {
    let target = format!("{}.msg", name);
    let mut subdirs = Vec::new();

    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .collect();
    entries.sort();

    for p in entries {
        if p.is_file() {
            if p.file_name().and_then(|n| n.to_str()) == Some(target.as_str()) {
                return Some(p);
            }
        } else if p.is_dir() {
            let skip = p.file_name().and_then(|n| n.to_str()).map(is_skipped_dir).unwrap_or(false);
            if !skip {
                subdirs.push(p);
            }
        }
    }

    if depth == 0 {
        return None;
    }
    subdirs.into_iter().find_map(|d| search_for_msg(&d, name, depth - 1))
}

// ---------------------------------------------------------------------------
// .msg parsing
// ---------------------------------------------------------------------------

/// `owning_pkg` is the package this file belongs to (known from the resolver),
/// used to qualify unqualified nested types. No path sniffing.
pub fn parse_msg_file(resolver: &MsgResolver, msg_path: &Path, owning_pkg: &str) -> Result<Vec<MsgField>> {
    let content = std::fs::read_to_string(msg_path)
        .with_context(|| format!("Failed to read {}", msg_path.display()))?;
    parse_msg_str(resolver, &content, owning_pkg, msg_path)
}

fn parse_msg_str(resolver: &MsgResolver, content: &str, owning_pkg: &str, source: &Path) -> Result<Vec<MsgField>> {
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
        if line.contains('=') && !line.split_whitespace().next().map_or(false, |t| t.contains("<=")) {
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

        // Split "TYPE[...]" into base and array spec
        let (base_raw, array_spec) = match type_token.find('[') {
            Some(b) => (&type_token[..b], Some(type_token[b + 1..].trim_end_matches(']'))),
            None => (type_token, None),
        };
        // Bounded strings: "string<=10" -> "string"
        let base_type_str = base_raw.split("<=").next().unwrap_or(base_raw);

        // Dynamic ("[]") and bounded ("[<=N]") arrays aren't supported for fixed-size CDR.
        let array_len = match array_spec {
            None => None,
            Some(spec) if spec.is_empty() || spec.starts_with("<=") => {
                eprintln!(
                    "WARNING: field '{}' is a dynamic array - skipping (not supported for fixed-size CDR)",
                    field_name
                );
                continue;
            }
            Some(spec) => Some(spec.parse::<usize>().with_context(|| {
                format!("Invalid array length in '{}' at {}", type_token, source.display())
            })?),
        };

        if let Some(prim) = parse_primitive(base_type_str) {
            let field_type = match array_len {
                Some(n) => FieldType::PrimArray(prim, n),
                None => FieldType::Primitive(prim),
            };
            fields.push(MsgField { name: field_name, field_type });
        } else {
            let nested_type_str = resolve_nested_type(base_type_str, owning_pkg);
            let resolved = resolver
                .find_msg_file(&nested_type_str)
                .with_context(|| format!("Resolving nested type '{}' for field '{}'", base_type_str, field_name))?;
            let nested_fields = parse_msg_file(resolver, &resolved.path, &resolved.pkg)?;
            let c_type_name = ros_type_to_c_struct_name(&nested_type_str);

            let field_type = match array_len {
                Some(n) => FieldType::NestedArray(c_type_name, nested_fields, n),
                None => FieldType::Nested(c_type_name, nested_fields),
            };
            fields.push(MsgField { name: field_name, field_type });
        }
    }

    Ok(fields)
}

/// Qualify a nested type reference as "pkg/msg/Name".
/// - Already qualified ("pkg/Name" or "pkg/msg/Name"): normalized to include "msg".
/// - "Header" is special in ROS: it always means std_msgs/Header.
/// - Anything else unqualified belongs to the package that references it.
fn resolve_nested_type(type_str: &str, owning_pkg: &str) -> String {
    let parts: Vec<&str> = type_str.split('/').collect();
    match parts.as_slice() {
        [pkg, name] => format!("{}/msg/{}", pkg, name),
        [_, _, _] => type_str.to_string(),
        _ if type_str == "Header" => "std_msgs/msg/Header".to_string(),
        _ => format!("{}/msg/{}", owning_pkg, type_str),
    }
}

/// "sensor_msgs/msg/Imu" -> "sensor_msgs__msg__Imu_t"  (rosidl_generator_c convention)
pub fn ros_type_to_c_struct_name(type_str: &str) -> String {
    let replaced = type_str.replace('/', "__");

    format!("{}_t", replaced)
}

/// "sensor_msgs/msg/Imu" -> "sensor_msgs__msg__Imu" (prefix for functions)
pub fn ros_type_to_c_prefix(type_str: &str) -> String {
    type_str.replace('/', "__")
}
