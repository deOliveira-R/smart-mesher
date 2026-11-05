//! Error classification: which Python exception class a protocol error maps to. The code
//! constants come from `sm-protocol`; nothing is duplicated by hand.

use sm_protocol::{
    APPLICATION_ERROR, INTERNAL_ERROR, INVALID_PARAMS, INVALID_REQUEST, METHOD_NOT_FOUND, PARSE_ERROR,
};

/// `kind` of a failed `script.run` (the statement error with the partial result in `data`).
pub const SCRIPT_ERROR_KIND: &str = "script-error";

/// Name of the Python exception class for a JSON-RPC error `code` and contract `kind`.
///
/// Unknown codes map to `SmartMeshError` itself.
pub fn class_name(code: i64, kind: &str) -> &'static str {
    match code {
        PARSE_ERROR => "ParseError",
        INVALID_REQUEST => "InvalidRequestError",
        METHOD_NOT_FOUND => "MethodNotFoundError",
        INVALID_PARAMS => "InvalidParamsError",
        INTERNAL_ERROR => "InternalError",
        APPLICATION_ERROR if kind == SCRIPT_ERROR_KIND => "ScriptError",
        APPLICATION_ERROR => "ApplicationError",
        _ => "SmartMeshError",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_protocol::RpcError;

    #[test]
    fn every_rpc_error_constructor_has_its_documented_class() {
        let cases = [
            (RpcError::parse("x"), "ParseError"),
            (RpcError::invalid_request("x"), "InvalidRequestError"),
            (RpcError::method_not_found("x"), "MethodNotFoundError"),
            (RpcError::invalid_params("x"), "InvalidParamsError"),
            (RpcError::internal("x"), "InternalError"),
            (RpcError::application("no-mesh", "x"), "ApplicationError"),
            (RpcError::application("script-error", "x"), "ScriptError"),
        ];
        for (e, want) in cases {
            assert_eq!(class_name(e.code, &e.kind), want, "{}", e.kind);
        }
        assert_eq!(class_name(1, "x"), "SmartMeshError");
    }
}
