package com.acme.camera.preset;

import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.TriangulationJobRequest;
import com.acme.camera.core.Point3D;

import java.util.List;

/**
 * The known cube calibration example. {@code projectionJob} and {@code triangulationJob}
 * are ready-to-POST request bodies for {@code /api/jobs/projection} and
 * {@code /api/jobs/triangulation}; every cube corner projects inside the image in both
 * demo cameras.
 */
public record CubeDemo(String description,
                       List<Point3D> worldCorners,
                       ProjectionJobRequest projectionJob,
                       TriangulationJobRequest triangulationJob) {
}
