package com.acme.camera.validation;

import java.util.Map;

/**
 * Typed validation failure. Thrown before any computation starts; translated by the
 * global exception handler into an HTTP 422 with a structured error body.
 */
public class GeometryValidationException extends RuntimeException {

    private final ErrorType type;
    private final Map<String, Object> details;

    public GeometryValidationException(ErrorType type, String message) {
        this(type, message, Map.of());
    }

    public GeometryValidationException(ErrorType type, String message, Map<String, Object> details) {
        super(message);
        this.type = type;
        this.details = details;
    }

    public ErrorType type() {
        return type;
    }

    public Map<String, Object> details() {
        return details;
    }
}
