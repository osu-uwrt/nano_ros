//! rustc-style error reporting: every error has a code, points at the line in ros.toml or the .msg
//! file that caused it, and says how to fix it where we can tell.
//!
//! Codes:
//!   E0001  ros.toml doesn't parse or doesn't match the schema
//!   E0101  message package not found
//!   E0102  message not found in its package
//!   E0103  variable-length array with no cap
//!   E0104  msg_type isn't "pkg/msg/Name"
//!   E0105  max_len names a field that isn't a variable-length array
//!   E0106  max_len is bigger than the field's bound
//!   E0107  one message type used with different max_len caps
//!   E0108  topic name isn't a valid C identifier
//!   E0109  two topics generate the same name
//!   E0110  field type the codegen can't handle yet

use std::fmt;
use std::io::IsTerminal;
use std::ops::Range;
use std::path::Path;
use std::sync::Arc;

use annotate_snippets::{AnnotationKind, Group, Level, Renderer, Snippet};

/// A file's contents, shared by every span that points into it.
#[derive(Debug, Clone)]
pub struct Source {
    pub path: Arc<str>,
    pub text: Arc<str>,
}

impl Source {
    pub fn new(path: &Path, text: String) -> Self {
        Self { path: path.display().to_string().into(), text: text.into() }
    }

    pub fn span(&self, range: Range<usize>) -> Span {
        Span { source: self.clone(), range }
    }
}

#[derive(Debug, Clone)]
pub struct Span {
    pub source: Source,
    pub range: Range<usize>,
}

#[derive(Debug)]
struct Label {
    span: Span,
    text: String,
    primary: bool,
}

#[derive(Debug)]
pub struct Diagnostic {
    code: &'static str,
    title: String,
    labels: Vec<Label>,
    notes: Vec<String>,
    help: Vec<String>,
}

impl Diagnostic {
    pub fn error(code: &'static str, title: impl Into<String>) -> Self {
        Self { code, title: title.into(), labels: Vec::new(), notes: Vec::new(), help: Vec::new() }
    }

    /// The span the error is about. Call once.
    pub fn primary(mut self, span: &Span, text: impl Into<String>) -> Self {
        self.labels.push(Label { span: span.clone(), text: text.into(), primary: true });
        self
    }

    /// Another span that explains the error, like where a type was first used.
    pub fn secondary(mut self, span: &Span, text: impl Into<String>) -> Self {
        self.labels.push(Label { span: span.clone(), text: text.into(), primary: false });
        self
    }

    pub fn note(mut self, text: impl Into<String>) -> Self {
        self.notes.push(text.into());
        self
    }

    pub fn help(mut self, text: impl Into<String>) -> Self {
        self.help.push(text.into());
        self
    }

    pub fn render(&self, renderer: &Renderer) -> String {
        let mut group = Group::with_title(Level::ERROR.primary_title(self.title.as_str()).id(self.code));

        // Labels in the same file share a snippet so they render together
        let mut files: Vec<&Source> = Vec::new();
        for label in &self.labels {
            if !files.iter().any(|f| Arc::ptr_eq(&f.text, &label.span.source.text)) {
                files.push(&label.span.source);
            }
        }
        for file in files {
            let mut snippet = Snippet::source(&*file.text).path(&*file.path).fold(true);
            for label in self.labels.iter().filter(|l| Arc::ptr_eq(&l.span.source.text, &file.text)) {
                let kind = if label.primary { AnnotationKind::Primary } else { AnnotationKind::Context };
                let mut annotation = kind.span(label.span.range.clone());
                if !label.text.is_empty() {
                    annotation = annotation.label(label.text.as_str());
                }
                snippet = snippet.annotation(annotation);
            }
            group = group.element(snippet);
        }
        for note in &self.notes {
            group = group.element(Level::NOTE.message(note.as_str()));
        }

        let mut report = vec![group];
        for help in &self.help {
            report.push(Group::with_title(Level::HELP.secondary_title(help.as_str())));
        }
        renderer.render(&report)
    }
}

impl fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "error[{}]: {}", self.code, self.title)
    }
}

impl std::error::Error for Diagnostic {}

/// Raised once the errors that were found have been printed.
#[derive(Debug)]
pub struct Aborted(pub usize);

impl fmt::Display for Aborted {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let plural = if self.0 == 1 { "" } else { "s" };
        write!(f, "could not generate code due to {} previous error{}", self.0, plural)
    }
}

impl std::error::Error for Aborted {}

/// Color when stderr is a terminal, honoring NO_COLOR and CLICOLOR_FORCE.
pub fn renderer() -> Renderer {
    let force = std::env::var_os("CLICOLOR_FORCE").is_some_and(|v| v != "0");
    let no_color = std::env::var_os("NO_COLOR").is_some_and(|v| !v.is_empty());
    if force || (!no_color && std::io::stderr().is_terminal()) {
        Renderer::styled()
    } else {
        Renderer::plain()
    }
}

/// The closest candidate to `name`, if it's close enough to be a likely typo.
pub fn did_you_mean<'a>(name: &str, candidates: impl IntoIterator<Item = &'a str>) -> Option<&'a str> {
    let max = (name.chars().count() / 3).max(1);
    candidates
        .into_iter()
        .map(|c| (levenshtein(name, c), c))
        .filter(|&(d, _)| d <= max)
        .min()
        .map(|(_, c)| c)
}

fn levenshtein(a: &str, b: &str) -> usize {
    let b: Vec<char> = b.chars().collect();
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.chars().enumerate() {
        let mut prev = row[0];
        row[0] = i + 1;
        for (j, &cb) in b.iter().enumerate() {
            let cur = row[j + 1];
            row[j + 1] = if ca == cb { prev } else { 1 + prev.min(row[j]).min(row[j + 1]) };
            prev = cur;
        }
    }
    row[b.len()]
}
