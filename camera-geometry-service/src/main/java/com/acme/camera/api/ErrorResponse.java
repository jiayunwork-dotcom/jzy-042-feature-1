package com.acme.camera.api;

import java.util.Map;

/** Uniform structured error body: {"error": {"type", "message", "details"}}. */
public record ErrorResponse(ApiError error) {

    public record ApiError(String type, String message, Map<String, Object> details) {
    }
}
