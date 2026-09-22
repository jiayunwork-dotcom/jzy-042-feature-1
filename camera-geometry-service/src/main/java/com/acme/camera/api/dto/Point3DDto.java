package com.acme.camera.api.dto;

/** 3D point as received over the wire (boxed so missing coordinates are detectable). */
public record Point3DDto(Double x, Double y, Double z) {
}
