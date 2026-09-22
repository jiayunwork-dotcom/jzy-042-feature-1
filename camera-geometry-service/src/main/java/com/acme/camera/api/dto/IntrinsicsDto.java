package com.acme.camera.api.dto;

/** Intrinsics as received over the wire: boxed types so missing JSON fields stay null and can be rejected explicitly. */
public record IntrinsicsDto(Double fx, Double fy, Double cx, Double cy, Integer width, Integer height) {
}
