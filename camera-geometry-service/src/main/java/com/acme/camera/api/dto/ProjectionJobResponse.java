package com.acme.camera.api.dto;

import java.util.List;

/** Result of a batch projection job: per-point pixels plus the job-level out-of-bounds count. */
public record ProjectionJobResponse(List<ProjectedPoint> results, int totalPoints, int outOfBoundsCount) {
}
