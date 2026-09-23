package com.acme.camera.api.dto;

/**
 * One observation's recovered world-to-camera pose plus that view's reprojection quality
 * (recomputed through the service projection pipeline from the converged parameters).
 */
public record CalibratedPoseDto(int observation,
                                ExtrinsicsDto extrinsics,
                                double maxReprojectionError,
                                double meanReprojectionError) {
}
