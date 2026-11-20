//! Checks on a parsed ros.toml that need its spans: topic names, and resolving max_len caps for
//! variable-length arrays.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::diagnostics::{did_you_mean, Diagnostic, Source, Span};
use crate::msg_file_parser::{ros_type_to_c_struct_name, FieldType, MsgField, Seq};
use crate::ros_toml_schema::{NanoRosConfig, Topic};

fn is_c_identifier(s: &str) -> bool {
    let mut chars = s.chars();
    chars.next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Topic names end up in C identifiers (`nros_publish_<name>`, `<NAME>_ID`), so they have to be
/// valid ones and can't collide.
pub fn check_topic_names(config: &NanoRosConfig, toml: &Source) -> Vec<Diagnostic> {
    let mut errors = Vec::new();
    let mut seen: HashMap<String, &Topic> = HashMap::new();
    for topic in &config.topics {
        let name = topic.name.get_ref();
        let span = toml.span(topic.name.span());
        if !is_c_identifier(name) {
            let fixed: String = name.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
            let mut d = Diagnostic::error("E0108", format!("topic name `{name}` isn't a valid C identifier"))
                .primary(&span, format!("used in generated names like `nros_publish_{name}`"));
            d = if is_c_identifier(&fixed) {
                d.help(format!("use letters, digits and underscores, e.g. `{fixed}`"))
            } else {
                d.help("use letters, digits and underscores, starting with a letter")
            };
            errors.push(d);
            continue;
        }
        let id_macro = format!("{}_ID", name.to_uppercase());
        match seen.get(&id_macro) {
            Some(first) => errors.push(
                Diagnostic::error("E0109", format!("topics `{}` and `{name}` both generate `{id_macro}`", first.name.get_ref()))
                    .primary(&span, "second topic here")
                    .secondary(&toml.span(first.name.span()), "first topic here")
                    .help("rename one of them"),
            ),
            None => {
                seen.insert(id_macro, topic);
            }
        }
    }
    errors
}

/// Quotes a max_len key if TOML needs it to.
fn toml_key(path: &str) -> String {
    if path.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
        path.to_string()
    } else {
        format!("\"{path}\"")
    }
}

struct CapCtx<'a> {
    topic_name: &'a str,
    max_len: &'a BTreeMap<toml::Spanned<String>, toml::Spanned<usize>>,
    msg_type_span: Span,
    toml: &'a Source,
    used: HashSet<String>,
    seq_paths: Vec<String>,
    /// Fields that aren't variable-length, path -> declaration
    other_paths: HashMap<String, String>,
    errors: Vec<Diagnostic>,
}

impl CapCtx<'_> {
    fn resolve(&mut self, path: &str, decl: &str, decl_span: &Span, seq: &mut Seq) {
        self.seq_paths.push(path.to_string());
        let requested = self.max_len.get_key_value(path);
        if requested.is_some() {
            self.used.insert(path.to_string());
        }
        match (seq.bound, requested) {
            (_, Some((_, value))) if *value.get_ref() == 0 => self.errors.push(
                Diagnostic::error("E0106", format!("max_len for `{path}` is 0"))
                    .primary(&self.toml.span(value.span()), "must be at least 1"),
            ),
            (Some(bound), Some((_, value))) if *value.get_ref() > bound => self.errors.push(
                Diagnostic::error("E0106", format!("max_len for `{path}` is bigger than its bound of {bound}"))
                    .primary(&self.toml.span(value.span()), format!("can be at most {bound}"))
                    .secondary(decl_span, "bound declared here")
                    .help(format!("use {bound} or less, or remove it to use the bound")),
            ),
            (_, Some((_, value))) => seq.cap = *value.get_ref(),
            (Some(bound), None) => seq.cap = bound,
            (None, None) => {
                let key = toml_key(path);
                let help = if self.max_len.is_empty() {
                    format!("set a cap in ros.toml: max_len = {{ {key} = 64 }}")
                } else {
                    format!("add a cap to this topic's max_len: {key} = 64")
                };
                self.errors.push(
                    Diagnostic::error("E0103", format!("`{decl}` has no fixed length"))
                        .primary(decl_span, "firmware messages need a compile-time size")
                        .secondary(&self.msg_type_span, format!("used by topic `{}`", self.topic_name))
                        .help(help),
                );
            }
        }
    }

    fn walk(&mut self, fields: &mut [MsgField], prefix: &str) {
        for field in fields {
            let MsgField { name, field_type, decl, span } = field;
            let path = if prefix.is_empty() { name.clone() } else { format!("{prefix}.{name}") };
            match field_type {
                FieldType::PrimSeq(_, seq) => self.resolve(&path, decl, span, seq),
                FieldType::NestedSeq(_, nested, seq) => {
                    self.resolve(&path, decl, span, seq);
                    self.walk(nested, &path);
                }
                FieldType::Nested(_, nested) | FieldType::NestedArray(_, nested, _) => {
                    self.other_paths.insert(path.clone(), decl.clone());
                    self.walk(nested, &path);
                }
                FieldType::Primitive(_) | FieldType::PrimArray(..) => {
                    self.other_paths.insert(path, decl.clone());
                }
            }
        }
    }
}

/// Sets the cap on every variable-length array in the topic, from its `[<=N]` bound or the
/// topic's max_len, and reports max_len entries that don't match a field.
pub fn apply_caps(topic: &mut Topic, toml: &Source) -> Vec<Diagnostic> {
    let mut ctx = CapCtx {
        topic_name: topic.name.get_ref(),
        max_len: &topic.max_len,
        msg_type_span: toml.span(topic.msg_type.span()),
        toml,
        used: HashSet::new(),
        seq_paths: Vec::new(),
        other_paths: HashMap::new(),
        errors: Vec::new(),
    };
    ctx.walk(&mut topic.fields, "");

    for key in topic.max_len.keys().filter(|k| !ctx.used.contains(k.get_ref().as_str())) {
        let path = key.get_ref();
        let span = toml.span(key.span());
        let d = match ctx.other_paths.get(path) {
            Some(decl) => Diagnostic::error("E0105", format!("`{path}` isn't a variable-length array"))
                .primary(&span, format!("declared as `{decl}`, which already has a fixed size"))
                .help("remove this entry"),
            None => {
                let msg_type = topic.msg_type.get_ref();
                let mut d = Diagnostic::error("E0105", format!("no field `{path}` in `{msg_type}`"))
                    .primary(&span, "no such field");
                if let Some(s) = did_you_mean(path, ctx.seq_paths.iter().map(String::as_str)) {
                    d = d.help(format!("did you mean `{s}`?"));
                } else if ctx.seq_paths.is_empty() {
                    d = d.note(format!("`{msg_type}` has no variable-length arrays")).help("remove max_len");
                } else {
                    d = d.note(format!("variable-length arrays in `{msg_type}`: {}", ctx.seq_paths.join(", ")));
                }
                d
            }
        };
        ctx.errors.push(d);
    }
    ctx.errors
}

/// The caps used inside one message type, by path relative to that type.
fn collect_caps(fields: &[MsgField], prefix: &str, out: &mut Vec<(String, usize)>) {
    for field in fields {
        let path = if prefix.is_empty() { field.name.clone() } else { format!("{prefix}.{}", field.name) };
        match &field.field_type {
            FieldType::PrimSeq(_, seq) => out.push((path, seq.cap)),
            FieldType::NestedSeq(_, nested, seq) => {
                out.push((path.clone(), seq.cap));
                collect_caps(nested, &path, out);
            }
            FieldType::Nested(_, nested) | FieldType::NestedArray(_, nested, _) => collect_caps(nested, &path, out),
            _ => {}
        }
    }
}

/// Visits every message type in a topic (the topic's own type and every nested one) with its fields.
fn visit_types<'a>(c_type: &'a str, fields: &'a [MsgField], f: &mut impl FnMut(&'a str, &'a [MsgField])) {
    f(c_type, fields);
    for field in fields {
        match &field.field_type {
            FieldType::Nested(t, nested) | FieldType::NestedArray(t, nested, _) | FieldType::NestedSeq(t, nested, _) => {
                visit_types(t, nested, f)
            }
            _ => {}
        }
    }
}

fn format_caps(caps: &[(String, usize)]) -> String {
    caps.iter().map(|(p, c)| format!("{} = {c}", toml_key(p))).collect::<Vec<_>>().join(", ")
}

/// Each message type becomes one C struct, so every topic using a type needs the same caps for it.
pub fn check_cap_conflicts(config: &NanoRosConfig, toml: &Source) -> Vec<Diagnostic> {
    let mut errors = Vec::new();
    let mut seen: HashMap<String, (Vec<(String, usize)>, &Topic)> = HashMap::new();
    let mut reported = HashSet::new();
    for topic in &config.topics {
        let top_type = ros_type_to_c_struct_name(topic.msg_type.get_ref());
        visit_types(&top_type, &topic.fields, &mut |c_type, fields| {
            let mut caps = Vec::new();
            collect_caps(fields, "", &mut caps);
            if caps.is_empty() {
                return;
            }
            match seen.get(c_type) {
                Some((first_caps, first)) if *first_caps != caps && reported.insert(c_type.to_string()) => {
                    let ros_type = c_type.trim_end_matches("_t").replace("__", "/");
                    errors.push(
                        Diagnostic::error("E0107", format!("`{ros_type}` is used with different max_len caps"))
                            .primary(&toml.span(topic.msg_type.span()), format!("here it gets {}", format_caps(&caps)))
                            .secondary(&toml.span(first.msg_type.span()), format!("here it gets {}", format_caps(first_caps)))
                            .note("each message type becomes one C struct, so every topic using it needs the same caps")
                            .help(format!("give `{}` and `{}` the same max_len", first.name.get_ref(), topic.name.get_ref())),
                    );
                }
                Some(_) => {}
                None => {
                    seen.insert(c_type.to_string(), (caps, topic));
                }
            }
        });
    }
    errors
}
