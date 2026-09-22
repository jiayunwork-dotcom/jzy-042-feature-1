package com.acme.camera.api.dto;

/** Result of projecting a single point. */
public record SingleProjectionResponse(double u, double v, boolean inBounds) {
}
