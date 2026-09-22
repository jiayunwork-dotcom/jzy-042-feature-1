package com.acme.camera.api.dto;

/** One projected point of a batch projection job. */
public record ProjectedPoint(int index, double u, double v, boolean inBounds) {
}
