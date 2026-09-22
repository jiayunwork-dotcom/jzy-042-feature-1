package com.acme.camera.api.dto;

/** Single-point projection request. Uses the same projection pipeline as batch jobs. */
public record SingleProjectionRequest(IntrinsicsDto intrinsics, DistortionDto distortion, Point3DDto point) {
}
