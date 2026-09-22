package com.acme.camera.api.dto;

import java.util.List;

/**
 * Batch projection job: one intrinsics+distortion set plus a batch of camera-frame 3D points.
 * Distortion may be null (= no distortion).
 */
public record ProjectionJobRequest(IntrinsicsDto intrinsics, DistortionDto distortion, List<Point3DDto> points) {
}
