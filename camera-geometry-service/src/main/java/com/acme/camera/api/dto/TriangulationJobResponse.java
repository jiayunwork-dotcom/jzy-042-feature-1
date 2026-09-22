package com.acme.camera.api.dto;

import java.util.List;

/**
 * Result of a triangulation job. Job-level max/mean reprojection errors are computed over
 * the valid pairs only; both are null when no pair is valid.
 */
public record TriangulationJobResponse(String triangulationMethod,
                                       List<TriangulatedPair> pairs,
                                       int pairCount,
                                       int validPairCount,
                                       Double maxReprojectionError,
                                       Double meanReprojectionError) {
}
