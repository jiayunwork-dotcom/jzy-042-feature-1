package com.acme.camera.api.dto;

/** Brown-Conrady coefficients as received over the wire. A null distortion object means "no distortion". */
public record DistortionDto(Double k1, Double k2, Double p1, Double p2) {
}
