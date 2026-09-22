package com.acme.camera.api.dto;

/** World-to-camera pose as received over the wire: 3x3 rotation (rows) + 3-vector translation. */
public record ExtrinsicsDto(double[][] rotation, double[] translation) {
}
