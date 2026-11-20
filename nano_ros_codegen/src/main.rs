use std::collections::HashSet;
use std::env::args;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use anyhow::{anyhow, bail, Context, Result};

use crate::checks::{apply_caps, check_cap_conflicts, check_topic_names};
use crate::diagnostics::{did_you_mean, Aborted, Diagnostic, Source};
use crate::ros_toml_schema::NanoRosConfig;
mod checks;
mod diagnostics;
mod ros_toml_schema;
mod msg_file_parser;
mod codegen;

use msg_file_parser::*;
use codegen::*; 

fn parse_args(args: &[String]) -> Result<(PathBuf, PathBuf, PathBuf, PathBuf, Option<PathBuf>)> {
    let mut toml_path: Option<PathBuf> = None;
    let mut ros_prefix: Option<PathBuf> = None;
    let mut user_types_path: Option<PathBuf> = None;
    let mut out_dir: Option<PathBuf> = None;
    let mut depfile: Option<PathBuf> = None;

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
            "--out-dir" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| anyhow!("--out-dir requires a value"))?;
                out_dir = Some(PathBuf::from(val));
            }
            "--depfile" => {
                i += 1;
                let val = args
                    .get(i)
                    .ok_or_else(|| anyhow!("--depfile requires a value"))?;
                depfile = Some(PathBuf::from(val));
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

    let out_dir = out_dir.unwrap_or_else(|| {
        toml_path
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."))
            .join("generated")
    });

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

    Ok((toml_path, ros_prefix, user_types_path, out_dir, depfile))
}

/// Escapes a path for a Makefile-style depfile.
fn depfile_escape(path: &Path) -> String {
    path.display()
        .to_string()
        .replace('$', "$$")
        .replace('#', "\\#")
        .replace(' ', "\\ ")
}

/// Writes a Makefile-style depfile so the build system reruns codegen when ros.toml
/// or any .msg file it read changes.
fn write_depfile(depfile: &Path, target: &Path, deps: &[PathBuf]) -> Result<()> {
    let mut out = format!("{}:", depfile_escape(&std::path::absolute(target)?));
    for dep in deps {
        out.push_str(" \\\n  ");
        out.push_str(&depfile_escape(&std::path::absolute(dep)?));
    }
    out.push('\n');
    std::fs::write(depfile, out).with_context(|| format!("Writing depfile {}", depfile.display()))
}

/// Requires exactly "pkg/msg/Name".
pub fn is_valid_topic_type(type_path: &str) -> bool {
    matches!(
        type_path.split('/').collect::<Vec<_>>().as_slice(),
        [pkg, "msg", name] if !pkg.is_empty() && !name.is_empty()
    )
}

/// Old ros.toml field names, so a stale example gets pointed at the new one.
const RENAMED_FIELDS: &[(&str, &str)] = &[("direction", "topic_type"), ("ros_type", "msg_type")];

/// Turns a ros.toml parse error into a diagnostic, suggesting the right name for unknown fields.
fn toml_error(err: &toml::de::Error, source: &Source) -> Diagnostic {
    let message = err.message().trim();
    let mut d = Diagnostic::error("E0001", message);
    if let Some(span) = err.span() {
        d = d.primary(&source.span(span), "");
    }
    // serde's message is "unknown field `x`, expected one of `a`, `b`", or the same with "variant"
    if let Some(rest) = message.strip_prefix("unknown field `").or_else(|| message.strip_prefix("unknown variant `")) {
        let names: Vec<&str> = rest.split('`').step_by(2).collect();
        if let Some((unknown, expected)) = names.split_first() {
            let renamed = RENAMED_FIELDS.iter().find(|(old, _)| old == unknown).map(|(_, new)| *new);
            if let Some(new) = renamed.or_else(|| did_you_mean(unknown, expected.iter().copied())) {
                d = d.help(format!("did you mean `{new}`?"));
            }
        }
    }
    d
}

/// Parses ros.toml and every message it uses, collecting all the errors instead of stopping at
/// the first.
fn load_config(toml_path: &Path, resolver: &MsgResolver) -> Result<NanoRosConfig> {
    let text = std::fs::read_to_string(toml_path)
        .with_context(|| format!("Reading {}", toml_path.display()))?;
    let source = Source::new(toml_path, text);
    let mut config: NanoRosConfig = match toml::from_str(&source.text) {
        Ok(c) => c,
        Err(e) => return Err(report(vec![toml_error(&e, &source)])),
    };

    let mut errors = check_topic_names(&config, &source);
    for topic in &mut config.topics {
        let span = source.span(topic.msg_type.span());
        let msg_type = topic.msg_type.get_ref();
        if !is_valid_topic_type(msg_type) {
            errors.push(
                Diagnostic::error("E0104", format!("invalid message type `{msg_type}`"))
                    .primary(&span, "expected `pkg/msg/Name`")
                    .help("write it like `sensor_msgs/msg/Imu`"),
            );
            continue;
        }
        match resolver.parse_type(msg_type, &span) {
            Ok(fields) => {
                topic.fields = fields;
                errors.extend(apply_caps(topic, &source));
            }
            Err(e) => errors.push(e.downcast::<Diagnostic>()?),
        }
    }
    if errors.is_empty() {
        errors = check_cap_conflicts(&config, &source);
    }
    if !errors.is_empty() {
        return Err(report(errors));
    }
    Ok(config)
}

/// Prints the errors (once each, since a bad nested type can be hit by several topics).
fn report(errors: Vec<Diagnostic>) -> anyhow::Error {
    let renderer = diagnostics::renderer();
    let mut printed = HashSet::new();
    for error in &errors {
        let rendered = error.render(&renderer);
        if printed.insert(rendered.clone()) {
            eprintln!("{rendered}\n");
        }
    }
    Aborted(printed.len()).into()
}

fn run() -> Result<()> {
    let args: Vec<String> = args().collect();
    let (toml_path, ros_prefix, user_types, out_dir, depfile) = parse_args(&args)?;
    println!("toml_path: {}, ros_pref: {}, user_types_dir: {}, out_dir: {}", toml_path.display(), ros_prefix.display(), user_types.display(), out_dir.display());

    let resolver = MsgResolver::new(&ros_prefix, &user_types)?;
    let config = load_config(&toml_path, &resolver)?;

    println!("[nano_ros_codegen] Board: {}, Transport: {:?}", config.board.name, config.board.transport);

    generate_core_code(&config, &out_dir)?;

    if let Some(depfile) = depfile {
        let mut deps = vec![toml_path.clone()];
        deps.extend(resolver.read_files());
        write_depfile(&depfile, &out_dir.join(HEADER_FILE_NAME), &deps)?;
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("error: {err:#}");
            ExitCode::FAILURE
        }
    }
}
