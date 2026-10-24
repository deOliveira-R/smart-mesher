//! A recording `Endpoint` and a harness around `sm_cli::run`, shared by the test files.
#![allow(dead_code, missing_docs)]

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::io::Cursor;
use std::rc::Rc;

use serde_json::{Value, json};
use sm_cli::{Io, Settings, run};
use sm_op_api::{Hash256, Head};
use sm_protocol::{Endpoint, Request, RpcError, StatusResult};

pub type Reply = Box<dyn FnMut(&Request) -> Result<Value, RpcError>>;

/// Records every request; answers with `reply`.
pub struct Fake {
    pub log: Rc<RefCell<Vec<Request>>>,
    pub reply: Reply,
}

impl Endpoint for Fake {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        let r = (self.reply)(&req);
        self.log.borrow_mut().push(req);
        r
    }
}

pub fn h(tag: &str) -> Hash256 {
    Hash256::of(tag.as_bytes())
}

pub fn status_value() -> Value {
    let mut branches = BTreeMap::new();
    branches.insert("main".to_owned(), h("n2"));
    branches.insert("alt".to_owned(), h("n1"));
    let s = StatusResult::new(Head::Ref("main".into()), h("state"))
        .with_position(Some(h("n2")), 2)
        .with_branches(2, branches)
        .with_redo(1)
        .with_project(Some("proj".into()));
    serde_json::to_value(s).unwrap()
}

pub fn log_value() -> Value {
    json!([
        {"node": h("n2").to_string(), "parent": h("n1").to_string(), "depth": 2, "op": "mesh.size", "author": "it", "time": "2026-10-24T00:00:00Z", "message": "mesh.size"},
        {"node": h("n1").to_string(), "parent": null, "depth": 1, "op": "model.create_brick", "author": "it", "time": "2026-10-24T00:00:00Z", "message": "make a brick"},
    ])
}

/// Default canned replies for every method.
pub fn canned(req: &Request) -> Result<Value, RpcError> {
    Ok(match req {
        Request::Apply { .. } => json!({
            "node": h("n").to_string(), "state": h("s").to_string(),
            "delta": {"created": ["TopoVolume#1"], "removed": [], "outputs": {"volume": ["TopoVolume#1"]},
                      "touched_layers": ["model"], "successors": {}, "diagnostics": []}
        }),
        Request::Query { .. } => json!({"cells": 1000}),
        Request::ScriptRun { .. } => {
            json!({"state": h("s").to_string(), "output": "hello\n", "events": [], "diagnostics": []})
        }
        Request::Log { .. } => log_value(),
        Request::Hash { .. } => {
            json!({"state": h("s").to_string(), "layers": {"model": h("m").to_string(), "meshes": h("x").to_string()}})
        }
        Request::Export { path, format, .. } => {
            json!({"path": path, "format": format, "bytes": 42, "sha256": h("e").to_string()})
        }
        Request::Catalog => catalog_value(),
        _ => status_value(),
    })
}

pub fn catalog_value() -> Value {
    json!({
        "protocol": 0,
        "ops": [{
            "name": "mesh.size", "tool_name": "mesh__size", "version": 1,
            "schema": {"type": "object", "description": "Set the target size.\nSecond line.",
                       "required": ["targets"],
                       "properties": {
                           "targets": {"type": "array", "items": {"type": "string", "x-entity-kind": ["TopoVolume"]}, "description": "what to size", "x-positional": true},
                           "size": {"type": ["number", "null"], "description": "edge length"}}},
            "output_schema": {"type": "object"},
            "outputs": [{"name": "sized", "kinds": ["TopoVolume"], "cardinality": "many", "doc": "the sized volumes"}],
            "positional": "targets"
        }],
        "queries": [{"name": "mesh.info", "tool_name": "mesh__info", "schema": {"type": "object", "description": "Mesh counts."}, "output_schema": {"type": "object"}}],
        "batch_schema": {}
    })
}

pub struct Outcome {
    pub code: i32,
    pub out: String,
    pub err: String,
    pub requests: Vec<Request>,
    pub settings: Option<Settings>,
}

impl Outcome {
    pub fn json(&self) -> Value {
        serde_json::from_str(self.out.trim())
            .unwrap_or_else(|e| panic!("stdout is not JSON ({e}): {:?}", self.out))
    }
}

/// Run `smesh <args>` against `reply` with `stdin` as input.
pub fn go_with(args: &[&str], stdin: &str, reply: Reply) -> Outcome {
    let log = Rc::new(RefCell::new(Vec::new()));
    let log2 = Rc::clone(&log);
    let seen = Rc::new(RefCell::new(None));
    let seen2 = Rc::clone(&seen);
    let mut reply = Some(reply);
    let mut connect = move |s: &Settings| -> Result<Box<dyn Endpoint>, String> {
        *seen2.borrow_mut() = Some(s.clone());
        Ok(Box::new(Fake {
            log: Rc::clone(&log2),
            reply: reply.take().ok_or("connected twice")?,
        }))
    };
    let mut input = Cursor::new(stdin.as_bytes().to_vec());
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = {
        let mut io = Io {
            stdin: &mut input,
            stdout: &mut out,
            stderr: &mut err,
        };
        let argv: Vec<&str> = std::iter::once("smesh")
            .chain(args.iter().copied())
            .collect();
        run(argv, &mut connect, &mut io)
    };
    let requests = log.borrow().clone();
    let settings = seen.borrow().clone();
    Outcome {
        code,
        out: String::from_utf8(out).unwrap(),
        err: String::from_utf8(err).unwrap(),
        requests,
        settings,
    }
}

pub fn go(args: &[&str]) -> Outcome {
    go_with(args, "", Box::new(canned))
}
