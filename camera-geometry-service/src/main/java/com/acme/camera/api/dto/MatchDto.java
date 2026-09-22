package com.acme.camera.api.dto;

/** One matched pair: the pixel of the same physical point in view 1 and in view 2. */
public record MatchDto(Point2DDto view1, Point2DDto view2) {
}
