//! `nano_ros_codegen check <ros.toml>... [--bus NAME=BITRATE]`: checks that only make sense
//! across every board on the robot, which codegen for a single board can't see.
//!
//! - every board uses the same message type for a topic
//! - publisher and subscriber QoS on different boards can actually match
//! - board names are unique
//! - each CAN bus has room for the traffic the boards' `rate_hz` values add up to

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use anyhow::{anyhow, bail, Context, Result};

use crate::codegen::max_payload_size;
use crate::msg_file_parser::MsgResolver;
use crate::ros_toml_schema::{NanoRosConfig, Qos, Topic, TopicType, Transport};
use crate::validate_topic_type;

/// nano_ros frame header and CRC (nros_frame_header_t and NROS_CRC_LEN in protocol.h)
const NROS_FRAME_OVERHEAD: usize = 6 + 2;
/// Bus name for CAN boards that don't set `bus`
const DEFAULT_BUS: &str = "can";
/// Bitrate for buses not given with --bus (Titan's internal bus)
const DEFAULT_BITRATE: u64 = 1_000_000;
const WARN_LOAD: f64 = 0.70;

struct Board {
    path: String,
    config: NanoRosConfig,
}

#[derive(Default)]
struct Report {
    errors: usize,
    warnings: usize,
}

impl Report {
    fn print(&mut self, level: &str, title: &str, lines: &[String], help: Option<&str>) {
        if level == "error" {
            self.errors += 1;
        } else {
            self.warnings += 1;
        }
        eprintln!("{level}: {title}");
        for line in lines {
            eprintln!("  --> {line}");
        }
        if let Some(help) = help {
            eprintln!("  = help: {help}");
        }
        eprintln!();
    }
}

fn direction(t: &Topic) -> &'static str {
    match t.topic_type {
        TopicType::Publisher => "publishes",
        TopicType::Subscriber => "subscribes",
    }
}

/// (reliable, transient local) for each profile, matching qos_from_wire in the agent. None for
/// system_default, which depends on the RMW.
fn qos_props(q: Qos) -> Option<(bool, bool)> {
    match q {
        Qos::Default | Qos::Reliable => Some((true, false)),
        Qos::SensorData | Qos::BestEffort => Some((false, false)),
        Qos::TransientLocal => Some((true, true)),
        Qos::SystemDefault => None,
    }
}

fn qos_name(q: Qos) -> &'static str {
    match q {
        Qos::Default => "default",
        Qos::SensorData => "sensor_data",
        Qos::Reliable => "reliable",
        Qos::BestEffort => "best_effort",
        Qos::TransientLocal => "transient_local",
        Qos::SystemDefault => "system_default",
    }
}

fn check_board_names(boards: &[Board], report: &mut Report) {
    let mut seen: HashMap<&str, &str> = HashMap::new();
    for b in boards {
        if let Some(first) = seen.insert(&b.config.board.name, &b.path) {
            report.print(
                "error",
                &format!("two boards are both named `{}`", b.config.board.name),
                &[first.to_string(), b.path.clone()],
                Some("board names identify boards to the agent and in /diagnostics, so they have to be unique"),
            );
        }
    }
}

fn check_topics(boards: &[Board], report: &mut Report) {
    let mut by_name: BTreeMap<&str, Vec<(&Board, &Topic)>> = BTreeMap::new();
    for b in boards {
        for t in &b.config.topics {
            by_name.entry(&t.name).or_default().push((b, t));
        }
    }

    for (name, uses) in &by_name {
        if uses.iter().any(|(_, t)| t.msg_type != uses[0].1.msg_type) {
            let lines: Vec<String> = uses
                .iter()
                .map(|(b, t)| format!("{}: {} {}", b.path, direction(t), t.msg_type))
                .collect();
            report.print(
                "error",
                &format!("topic `{name}` has different message types on different boards"),
                &lines,
                Some("ROS never connects a publisher and subscriber with different types; use the same msg_type everywhere"),
            );
            continue;
        }

        for (pb, pt) in uses.iter().filter(|(_, t)| t.topic_type == TopicType::Publisher) {
            for (sb, st) in uses.iter().filter(|(_, t)| t.topic_type == TopicType::Subscriber) {
                let (Some((p_rel, p_tl)), Some((s_rel, s_tl))) = (qos_props(pt.qos), qos_props(st.qos)) else {
                    continue;
                };
                let why = if s_rel && !p_rel {
                    "the subscriber asks for reliable delivery but the publisher is best effort"
                } else if s_tl && !p_tl {
                    "the subscriber asks for transient local (latched) but the publisher is volatile"
                } else {
                    continue;
                };
                report.print(
                    "error",
                    &format!("QoS on topic `{name}` can't match: {why}"),
                    &[
                        format!("{}: publishes with qos = \"{}\"", pb.path, qos_name(pt.qos)),
                        format!("{}: subscribes with qos = \"{}\"", sb.path, qos_name(st.qos)),
                    ],
                    Some("ROS silently never delivers between incompatible QoS; give the subscriber a profile the publisher satisfies"),
                );
            }
        }
    }
}

/// Worst-case bits on the wire for one CAN 2.0 frame with `data` bytes, bit stuffing included
fn can_frame_bits(data: usize, extended: bool) -> u64 {
    let g = if extended { 54 } else { 34 };
    let bits = g + 8 * data as u64 + 13;
    bits + (g + 8 * data as u64 - 1) / 4
}

/// Worst-case bits canmore needs for a message of `len` bytes: 8-byte frames, the first and last
/// with extended IDs, or a single extended frame for a message that fits in one
fn canmore_message_bits(len: usize) -> u64 {
    if len <= 8 {
        return can_frame_bits(len, true);
    }
    let frames = len.div_ceil(8);
    let last = len - 8 * (frames - 1);
    can_frame_bits(8, true) + (frames as u64 - 2) * can_frame_bits(8, false) + can_frame_bits(last, true)
}

struct Load<'a> {
    board: &'a str,
    topic: &'a Topic,
    frame_bytes: usize,
    bits_per_s: f64,
}

fn check_buses(boards: &[Board], bitrates: &HashMap<String, u64>, report: &mut Report) {
    let mut buses: BTreeMap<&str, Vec<Load>> = BTreeMap::new();
    let mut unrated: BTreeMap<&str, usize> = BTreeMap::new();
    for b in boards.iter().filter(|b| b.config.board.transport == Transport::Can) {
        let bus = b.config.board.bus.as_deref().unwrap_or(DEFAULT_BUS);
        let loads = buses.entry(bus).or_default();
        for t in &b.config.topics {
            match t.rate_hz {
                Some(rate) => {
                    let frame_bytes = NROS_FRAME_OVERHEAD + max_payload_size(t);
                    loads.push(Load {
                        board: &b.config.board.name,
                        topic: t,
                        frame_bytes,
                        bits_per_s: rate * canmore_message_bits(frame_bytes) as f64,
                    });
                }
                None => *unrated.entry(bus).or_default() += 1,
            }
        }
    }

    for (bus, mut loads) in buses {
        let (bitrate, assumed) = match bitrates.get(bus) {
            Some(r) => (*r, ""),
            None => (DEFAULT_BITRATE, ", assumed; set it with --bus"),
        };
        if loads.is_empty() {
            let n = unrated.get(bus).copied().unwrap_or(0);
            println!("CAN bus `{bus}`: none of its {n} topic(s) set rate_hz, so there's nothing to budget\n");
            continue;
        }
        loads.sort_by(|a, b| b.bits_per_s.total_cmp(&a.bits_per_s));
        let total = loads.iter().map(|l| l.bits_per_s).fold(0.0, |a, b| a + b) / bitrate as f64;

        println!("CAN bus `{bus}` at {} kbit/s{assumed}", bitrate / 1000);
        println!("  {:<16} {:<24} {:>4} {:>8} {:>10} {:>7}", "board", "topic", "dir", "rate Hz", "max bytes", "load");
        for l in &loads {
            let dir = if l.topic.topic_type == TopicType::Publisher { "out" } else { "in" };
            println!(
                "  {:<16} {:<24} {:>4} {:>8} {:>10} {:>6.1}%",
                l.board,
                l.topic.name,
                dir,
                l.topic.rate_hz.unwrap_or(0.0),
                l.frame_bytes,
                100.0 * l.bits_per_s / bitrate as f64
            );
        }
        println!("  {:<16} {:<24} {:>4} {:>8} {:>10} {:>6.1}%", "total", "", "", "", "", 100.0 * total);
        if let Some(n) = unrated.get(bus) {
            println!("  ({n} topic{} without rate_hz not counted)", if *n == 1 { "" } else { "s" });
        }
        println!("  worst case: every message at its largest, with full CAN bit stuffing\n");

        let title = format!("CAN bus `{bus}` is at {:.0}% of {} kbit/s", 100.0 * total, bitrate / 1000);
        let help = "lower rate_hz on the biggest topics, use smaller messages, or move boards to another bus";
        if total > 1.0 {
            report.print("error", &title, &[], Some(help));
        } else if total >= WARN_LOAD {
            report.print("warning", &format!("{title}, little room left"), &[], Some(help));
        }
    }
}

fn parse_bus(spec: &str) -> Result<(String, u64)> {
    let (name, rate) = spec
        .split_once('=')
        .ok_or_else(|| anyhow!("--bus expects NAME=BITRATE, e.g. internal=1000000"))?;
    let rate: u64 = rate.parse().with_context(|| format!("--bus {spec}: bitrate isn't a number"))?;
    if rate == 0 {
        bail!("--bus {spec}: bitrate can't be 0");
    }
    Ok((name.to_string(), rate))
}

/// Runs the checks. Returns an error if any check failed.
pub fn run(args: &[String]) -> Result<()> {
    let mut tomls = Vec::new();
    let mut ros_prefix: Option<PathBuf> = None;
    let mut user_msgs: Option<PathBuf> = None;
    let mut bitrates = HashMap::new();
    let mut i = 0;
    while i < args.len() {
        let value = |i: usize| args.get(i + 1).ok_or_else(|| anyhow!("{} requires a value", args[i]));
        match args[i].as_str() {
            "--ros-prefix" => ros_prefix = Some(PathBuf::from(value(i)?)),
            "--user-msgs" => user_msgs = Some(PathBuf::from(value(i)?)),
            "--bus" => {
                let (name, rate) = parse_bus(value(i)?)?;
                bitrates.insert(name, rate);
            }
            arg if !arg.starts_with("--") => {
                tomls.push(PathBuf::from(arg));
                i += 1;
                continue;
            }
            other => bail!("Unknown argument: {other}"),
        }
        i += 2;
    }
    if tomls.is_empty() {
        bail!("usage: nano_ros_codegen check <ros.toml>... [--ros-prefix DIR] [--user-msgs DIR] [--bus NAME=BITRATE]");
    }

    let ros_prefix = ros_prefix
        .or_else(|| std::env::var("AMENT_PREFIX_PATH").ok().map(|p| PathBuf::from(p.split(':').next().unwrap_or(""))))
        .unwrap_or_else(|| PathBuf::from("/opt/ros/humble"));
    // the resolver wants a user directory; with none, searching the ROS prefix again finds nothing new
    let resolver = MsgResolver::new(&ros_prefix, user_msgs.as_deref().unwrap_or(&ros_prefix))?;

    let mut boards = Vec::new();
    let mut report = Report::default();
    for path in &tomls {
        let text = std::fs::read_to_string(path).with_context(|| format!("Reading {}", path.display()))?;
        let mut config: NanoRosConfig =
            toml::from_str(&text).with_context(|| format!("Parsing {}", path.display()))?;
        for topic in &mut config.topics {
            validate_topic_type(&topic.msg_type).with_context(|| format!("{}: topic '{}'", path.display(), topic.name))?;
            topic.fields = resolver
                .parse_type(&topic.msg_type)
                .with_context(|| format!("{}: resolving '{}'", path.display(), topic.msg_type))?;
            if topic.rate_hz.is_some_and(|r| !(r > 0.0)) {
                report.print(
                    "error",
                    &format!("topic `{}` has a rate_hz that isn't positive", topic.name),
                    &[path.display().to_string()],
                    None,
                );
            }
        }
        boards.push(Board { path: path.display().to_string(), config });
    }

    check_board_names(&boards, &mut report);
    check_topics(&boards, &mut report);
    check_buses(&boards, &bitrates, &mut report);

    if report.errors > 0 {
        bail!("{} error(s), {} warning(s) across {} board(s)", report.errors, report.warnings, boards.len());
    }
    println!("ok: {} board(s), {} warning(s)", boards.len(), report.warnings);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_frame_bits_match_worst_case_formula() {
        // the usual numbers for 8 data bytes: 135 bits with an 11-bit ID, 160 with a 29-bit one
        assert_eq!(can_frame_bits(8, false), 135);
        assert_eq!(can_frame_bits(8, true), 160);
    }

    #[test]
    fn canmore_splits_into_8_byte_frames() {
        assert_eq!(canmore_message_bits(5), can_frame_bits(5, true));
        assert_eq!(canmore_message_bits(16), 2 * can_frame_bits(8, true));
        // first frame extended, middle standard, a 4-byte extended last frame
        assert_eq!(canmore_message_bits(20), can_frame_bits(8, true) + can_frame_bits(8, false) + can_frame_bits(4, true));
    }
}
