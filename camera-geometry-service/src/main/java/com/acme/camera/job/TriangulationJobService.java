package com.acme.camera.job;

import com.acme.camera.api.dto.CameraDto;
import com.acme.camera.api.dto.MatchDto;
import com.acme.camera.api.dto.TriangulatedPair;
import com.acme.camera.api.dto.TriangulationJobRequest;
import com.acme.camera.api.dto.TriangulationJobResponse;
import com.acme.camera.core.CameraModel;
import com.acme.camera.core.DltTriangulator;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point2D;
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
 * Orchestrates triangulation jobs. Stateless, so concurrent jobs stay isolated.
 *
 * <p>Method: DLT (see {@link DltTriangulator}). For each matched pair the 3D point is
 * triangulated, then reprojected into BOTH views through the full projection pipeline
 * (including distortion); the reprojection error of a pair is the mean of the two
 * per-view pixel distances. Job-level max/mean are taken over the valid pairs.
 *
 * <p>Input validation is fail-fast (whole job rejected before any computation). A pair
 * whose triangulated point lands at/behind a camera is a computational outcome, not an
 * input error: that pair is flagged {@code valid=false} (with null error fields) and the
 * remaining pairs are still processed.
 */
@Service
public class TriangulationJobService {

    public static final String METHOD = "DLT";

    private final DltTriangulator triangulator;
    private final PinholeProjector projector;
    private final JobMetrics metrics;

    public TriangulationJobService(DltTriangulator triangulator, PinholeProjector projector, JobMetrics metrics) {
        this.triangulator = triangulator;
        this.projector = projector;
        this.metrics = metrics;
    }

    public TriangulationJobResponse execute(TriangulationJobRequest request) {
        if (request == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD, "Missing request body");
        }
        CameraModel camera1 = requireCamera(request.camera1(), "camera1");
        CameraModel camera2 = requireCamera(request.camera2(), "camera2");

        List<MatchDto> matchDtos = request.matches();
        if (matchDtos == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing required field 'matches'", Map.of("field", "matches"));
        }
        if (matchDtos.isEmpty()) {
            throw new GeometryValidationException(ErrorType.EMPTY_MATCH_SET,
                    "Triangulation job carries zero matched pairs");
        }

        // Fail-fast policy: validate every matched pixel before triangulating anything.
        List<Point2D> pixels1 = new ArrayList<>(matchDtos.size());
        List<Point2D> pixels2 = new ArrayList<>(matchDtos.size());
        for (int i = 0; i < matchDtos.size(); i++) {
            MatchDto match = matchDtos.get(i);
            if (match == null) {
                throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                        "Missing match at 'matches[" + i + "]'", Map.of("field", "matches[" + i + "]"));
            }
            pixels1.add(InputValidator.requirePixel(match.view1(), "matches[" + i + "].view1"));
            pixels2.add(InputValidator.requirePixel(match.view2(), "matches[" + i + "].view2"));
        }

        List<TriangulatedPair> pairs = new ArrayList<>(matchDtos.size());
        double maxError = 0.0;
        double errorSum = 0.0;
        int validCount = 0;

        for (int i = 0; i < pixels1.size(); i++) {
            Point3D world = triangulator.triangulate(pixels1.get(i), camera1, pixels2.get(i), camera2);

            Point3D inCam1 = camera1.extrinsics().transform(world);
            Point3D inCam2 = camera2.extrinsics().transform(world);
            boolean reprojectable = Double.isFinite(world.x()) && Double.isFinite(world.y())
                    && Double.isFinite(world.z())
                    && inCam1.z() > 0.0 && inCam2.z() > 0.0;
            if (!reprojectable) {
                pairs.add(new TriangulatedPair(i, world, false, null, null, null));
                continue;
            }

            ProjectionResult r1 = projector.project(inCam1, camera1.intrinsics(), camera1.distortion());
            ProjectionResult r2 = projector.project(inCam2, camera2.intrinsics(), camera2.distortion());
            double error1 = Math.hypot(r1.u() - pixels1.get(i).x(), r1.v() - pixels1.get(i).y());
            double error2 = Math.hypot(r2.u() - pixels2.get(i).x(), r2.v() - pixels2.get(i).y());
            double pairError = 0.5 * (error1 + error2);

            pairs.add(new TriangulatedPair(i, world, true, error1, error2, pairError));
            maxError = Math.max(maxError, pairError);
            errorSum += pairError;
            validCount++;
        }

        Double max = validCount > 0 ? maxError : null;
        Double mean = validCount > 0 ? errorSum / validCount : null;
        metrics.triangulationJobCompleted();
        return new TriangulationJobResponse(METHOD, pairs, pairs.size(), validCount, max, mean);
    }

    private static CameraModel requireCamera(CameraDto dto, String path) {
        if (dto == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing camera at '" + path + "'", Map.of("field", path));
        }
        return new CameraModel(
                InputValidator.requireIntrinsics(dto.intrinsics(), path + ".intrinsics"),
                InputValidator.requireDistortion(dto.distortion(), path + ".distortion"),
                InputValidator.requireExtrinsics(dto.extrinsics(), path + ".extrinsics"));
    }
}
