package com.acme.camera.api;

import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.HttpMessageNotReadableException;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;

import java.util.Map;

/**
 * Turns every failure into a structured, typed error body — no stack traces, no empty
 * responses, no uncaught exceptions leaking out of the service.
 */
@RestControllerAdvice
public class GlobalExceptionHandler {

    /** Domain validation failures: 422 with the typed error. */
    @ExceptionHandler(GeometryValidationException.class)
    public ResponseEntity<ErrorResponse> handleValidation(GeometryValidationException ex) {
        return ResponseEntity.status(HttpStatus.UNPROCESSABLE_ENTITY)
                .body(new ErrorResponse(new ErrorResponse.ApiError(
                        ex.type().name(), ex.getMessage(), ex.details())));
    }

    /** Unparseable or wrongly shaped JSON: 400. */
    @ExceptionHandler(HttpMessageNotReadableException.class)
    public ResponseEntity<ErrorResponse> handleMalformed(HttpMessageNotReadableException ex) {
        return ResponseEntity.status(HttpStatus.BAD_REQUEST)
                .body(new ErrorResponse(new ErrorResponse.ApiError(
                        ErrorType.MALFORMED_REQUEST.name(),
                        "Request body is not valid JSON or does not match the expected shape",
                        Map.of())));
    }

    /** Last line of defense: never leak an uncaught exception. */
    @ExceptionHandler(Exception.class)
    public ResponseEntity<ErrorResponse> handleUnexpected(Exception ex) {
        return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR)
                .body(new ErrorResponse(new ErrorResponse.ApiError(
                        ErrorType.INTERNAL_ERROR.name(),
                        "Unexpected internal error",
                        Map.of())));
    }
}
