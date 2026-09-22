package com.acme.camera.job;

import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.api.dto.ProjectedPoint;
import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.ProjectionJobResponse;
import com.acme.camera.api.dto.SingleProjectionRequest;
import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.acme.camera.core.ProjectionResult;
import com.acme.camera.validation.ErrorType;
import com.acme.camera.validation.GeometryValidationException;
import com.acme.camera.validation.InputValidator;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Orchestrates projection work. Stateless: every request is validated and computed from
 * its own local data, so any number of jobs may run concurrently without interference.
 *
 * <p>Invalid-point policy (consistent across the service): <b>fail fast, reject the whole
 * job</b>. Every point is validated before the first projection happens; a single illegal
 * point (e.g. Z &lt;= 0) aborts the job with a typed 422 error rather than producing a
 * partial result.
 */
@Service
public class ProjectionJobService {

    private final PinholeProjector projector;
    private final JobMetrics metrics;

    public ProjectionJobService(PinholeProjector projector, JobMetrics metrics) {
        this.projector = projector;
        this.metrics = metrics;
    }

    /** Batch projection job. */
    public ProjectionJobResponse execute(ProjectionJobRequest request) {
        if (request == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD, "Missing request body");
        }
        Intrinsics intrinsics = InputValidator.requireIntrinsics(request.intrinsics(), "intrinsics");
        DistortionCoefficients distortion = InputValidator.requireDistortion(request.distortion(), "distortion");
        List<Point3DDto> pointDtos = request.points();
        if (pointDtos == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing required field 'points'", Map.of("field", "points"));
        }

        // Fail-fast policy: validate ALL points before projecting anything.
        List<Point3D> points = new ArrayList<>(pointDtos.size());
        for (int i = 0; i < pointDtos.size(); i++) {
            points.add(InputValidator.requireProjectablePoint(pointDtos.get(i), "points[" + i + "]"));
        }

        List<ProjectedPoint> results = new ArrayList<>(points.size());
        int outOfBounds = 0;
        for (int i = 0; i < points.size(); i++) {
            ProjectionResult r = projector.project(points.get(i), intrinsics, distortion);
            if (!r.inBounds()) {
                outOfBounds++;
            }
            results.add(new ProjectedPoint(i, r.u(), r.v(), r.inBounds()));
        }
        metrics.projectionJobCompleted();
        return new ProjectionJobResponse(results, results.size(), outOfBounds);
    }

    /** Single-point projection — deliberately routed through the same projector as batch jobs. */
    public ProjectionResult projectSingle(SingleProjectionRequest request) {
        if (request == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD, "Missing request body");
        }
        Intrinsics intrinsics = InputValidator.requireIntrinsics(request.intrinsics(), "intrinsics");
        DistortionCoefficients distortion = InputValidator.requireDistortion(request.distortion(), "distortion");
        Point3D point = InputValidator.requireProjectablePoint(request.point(), "point");
        return projector.project(point, intrinsics, distortion);
    }
}
