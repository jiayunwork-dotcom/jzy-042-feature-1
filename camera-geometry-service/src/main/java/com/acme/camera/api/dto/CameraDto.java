package com.acme.camera.api.dto;

/** One camera of a triangulation job. Distortion may be null (= no distortion). */
public record CameraDto(IntrinsicsDto intrinsics, DistortionDto distortion, ExtrinsicsDto extrinsics) {
}
