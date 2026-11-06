use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use anyhow::{Context, Result, anyhow};

use crate::diagnostics::{did_you_mean, Diagnostic, Source, Span};

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
    /// Variable-length array of a primitive: float32[] or float32[<=N]
    PrimSeq(PrimType, Seq),
    /// Variable-length array of a nested type
    NestedSeq(String, Vec<MsgField>, Seq),
}

/// Length of a variable-length array. Firmware stores up to `cap` elements inline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seq {
    /// N from "[<=N]", None for "[]"
    pub bound: Option<usize>,
    /// Set from the bound or ros.toml's max_len after parsing; 0 until then
    pub cap: usize,
}

/// A single field in a .msg definition, after resolution.
#[derive(Debug, Clone)]
pub struct MsgField {
    pub name: String,
    pub field_type: FieldType,
    /// The declaration as written, e.g. "float32[] data"
    pub decl: String,
    /// Where the declaration is in its .msg file
    pub span: Span,
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
    /// The directories searched, for error messages
    roots: Vec<PathBuf>,
}

/// Why a "pkg/msg/Name" couldn't be resolved to a file.
#[derive(Debug)]
pub enum ResolveError {
    BadPath,
    NoPackage { pkg: String },
    NoMsg { pkg: String, name: String, pkg_dir: PathBuf },
}

/// Result of resolving "pkg/Name" to an actual file.
pub struct ResolvedMsg {
    pub path: PathBuf,
    pub pkg: String,
}

impl MsgResolver {
    /// `ros_share` is e.g. `/opt/ros/humble/share` (no `.join("share")` anymore).
    pub fn new(ros_share: &Path, user_types_path: &Path) -> Result<Self> {
        let mut roots = vec![ros_share.to_path_buf()];
        let user_pkgs = if user_types_path.as_os_str().is_empty() {
            PackageIndex::new()
        } else {
            roots.push(user_types_path.to_path_buf());
            build_package_index(user_types_path)
                .with_context(|| format!("Indexing user packages in {}", user_types_path.display()))?
        };
        Ok(Self {
            ros_pkgs: build_package_index(ros_share)
                .with_context(|| format!("Indexing ROS packages in {}", ros_share.display()))?,
            user_pkgs,
            roots,
        })
    }

    /// Turns a resolve failure for `type_path` into an error pointing at `span`.
    pub fn diagnose(&self, err: ResolveError, type_path: &str, span: &Span) -> Diagnostic {
        match err {
            ResolveError::BadPath => Diagnostic::error("E0104", format!("invalid message type `{type_path}`"))
                .primary(span, "expected `pkg/msg/Name`")
                .help("write it like `sensor_msgs/msg/Imu`"),
            ResolveError::NoPackage { pkg } => {
                let roots: Vec<String> = self.roots.iter().map(|r| r.display().to_string()).collect();
                let mut d = Diagnostic::error("E0101", format!("no message package `{pkg}`"))
                    .primary(span, format!("not found in {}", roots.join(" or ")));
                let known = self.ros_pkgs.keys().chain(self.user_pkgs.keys()).map(String::as_str);
                if let Some(suggestion) = did_you_mean(&pkg, known) {
                    d = d.help(format!("did you mean `{suggestion}`?"));
                } else if self.roots.len() == 1 {
                    d = d.help("if it's a custom package, pass the directory it's in with --user-msgs");
                }
                d
            }
            ResolveError::NoMsg { pkg, name, pkg_dir } => {
                let mut d = Diagnostic::error("E0102", format!("no message `{name}` in package `{pkg}`"))
                    .primary(span, format!("not found in {}", pkg_dir.display()));
                let msgs = list_msgs(&pkg_dir);
                if let Some(suggestion) = did_you_mean(&name, msgs.iter().map(String::as_str)) {
                    d = d.help(format!("did you mean `{pkg}/msg/{suggestion}`?"));
                } else if did_you_mean(&name, ["Header"]).is_some() {
                    d = d.help("did you mean `Header` (std_msgs/msg/Header)?");
                }
                d
            }
        }
    }

    /// Resolve "pkg/Name" or "pkg/msg/Name" to a .msg file. ROS install wins over user types,
    /// but if the ROS package exists and lacks the message we still fall back to user types.
    pub fn find_msg_file(&self, type_path: &str) -> Result<ResolvedMsg, ResolveError> {
        let (pkg, name) = split_type_path(type_path).map_err(|_| ResolveError::BadPath)?;

        let candidates = [self.ros_pkgs.get(pkg), self.user_pkgs.get(pkg)];
        if candidates.iter().all(|c| c.is_none()) {
            return Err(ResolveError::NoPackage { pkg: pkg.to_string() });
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

        let pkg_dir = candidates.into_iter().flatten().next().cloned().unwrap_or_default();
        Err(ResolveError::NoMsg { pkg: pkg.to_string(), name: name.to_string(), pkg_dir })
    }

    /// Convenience entry point: "riptide_msgs2/FirmwareStatus" -> parsed fields. `span` is where the
    /// type was named, for errors.
    pub fn parse_type(&self, type_path: &str, span: &Span) -> Result<Vec<MsgField>> {
        let resolved = self.find_msg_file(type_path).map_err(|e| self.diagnose(e, type_path, span))?;
        parse_msg_file(self, &resolved.path, &resolved.pkg)
    }
}

/// Message names in a package, for suggestions.
fn list_msgs(pkg_dir: &Path) -> Vec<String> {
    std::fs::read_dir(pkg_dir.join("msg"))
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok())
        .filter_map(|e| e.file_name().to_str()?.strip_suffix(".msg").map(str::to_string))
        .collect()
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
    parse_msg_str(resolver, &Source::new(msg_path, content), owning_pkg)
}

fn parse_msg_str(resolver: &MsgResolver, source: &Source, owning_pkg: &str) -> Result<Vec<MsgField>> {
    let mut fields = Vec::new();
    let mut line_start = 0;
    for raw_line in source.text.split_inclusive('\n') {
        let offset = line_start;
        line_start += raw_line.len();

        let code = raw_line.split('#').next().unwrap_or("");
        let line = code.trim();
        if line.is_empty() {
            continue;
        }
        let line_offset = offset + (code.len() - code.trim_start().len());

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
        let decl = format!("{type_token} {field_name}");
        let decl_span = source.span(line_offset..line_offset + line.len());
        let type_span = source.span(line_offset..line_offset + type_token.len());

        // Split "TYPE[...]" into base and array spec
        let (base_raw, array_spec) = match type_token.find('[') {
            Some(b) => (&type_token[..b], Some(type_token[b + 1..].trim_end_matches(']'))),
            None => (type_token, None),
        };
        // Bounded strings: "string<=10" -> "string"
        let base_type_str = base_raw.split("<=").next().unwrap_or(base_raw);

        enum Len {
            One,
            Fixed(usize),
            Seq(Seq),
        }
        let bad_len = || {
            Diagnostic::error("E0110", format!("invalid array length in `{decl}`"))
                .primary(&type_span, "expected `[N]`, `[<=N]` or `[]`")
        };
        let len = match array_spec {
            None => Len::One,
            Some("") => Len::Seq(Seq { bound: None, cap: 0 }),
            Some(spec) => match spec.strip_prefix("<=") {
                Some(n) => Len::Seq(Seq { bound: Some(n.parse().map_err(|_| bad_len())?), cap: 0 }),
                None => Len::Fixed(spec.parse().map_err(|_| bad_len())?),
            },
        };

        if let Some(prim) = parse_primitive(base_type_str) {
            if prim == PrimType::String && !matches!(len, Len::One) {
                return Err(Diagnostic::error("E0110", format!("arrays of strings aren't supported yet: `{decl}`"))
                    .primary(&decl_span, "firmware strings are fixed `char[256]` buffers")
                    .into());
            }
            let field_type = match len {
                Len::One => FieldType::Primitive(prim),
                Len::Fixed(n) => FieldType::PrimArray(prim, n),
                Len::Seq(seq) => FieldType::PrimSeq(prim, seq),
            };
            fields.push(MsgField { name: field_name, field_type, decl, span: decl_span });
        } else {
            let nested_type_str = resolve_nested_type(base_type_str, owning_pkg);
            let nested_fields = resolver.parse_type(&nested_type_str, &type_span)?;
            let c_type_name = ros_type_to_c_struct_name(&nested_type_str);

            let field_type = match len {
                Len::One => FieldType::Nested(c_type_name, nested_fields),
                Len::Fixed(n) => FieldType::NestedArray(c_type_name, nested_fields, n),
                Len::Seq(seq) => FieldType::NestedSeq(c_type_name, nested_fields, seq),
            };
            fields.push(MsgField { name: field_name, field_type, decl, span: decl_span });
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
