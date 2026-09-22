package com.acme.camera.core;

/** A full camera: intrinsics + distortion + world-to-camera pose. */
public record CameraModel(Intrinsics intrinsics, DistortionCoefficients distortion, Extrinsics extrinsics) {
}
