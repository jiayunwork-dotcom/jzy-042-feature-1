package com.acme.camera.api.dto;

/** 2D pixel as received over the wire (boxed so missing coordinates are detectable). */
public record Point2DDto(Double x, Double y) {
}
