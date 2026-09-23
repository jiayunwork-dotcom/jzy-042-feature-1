package com.acme.camera.job;

import com.acme.camera.api.dto.CalibratedPoseDto;
import com.acme.camera.api.dto.CalibrationJobRequest;
import com.acme.camera.api.dto.CalibrationJobResponse;
import com.acme.camera.api.dto.CalibrationObservationDto;
import com.acme.camera.api.dto.ConvergenceDto;
import com.acme.camera.api.dto.DistortionDto;
import com.acme.camera.api.dto.ExtrinsicsDto;
import com.acme.camera.api.dto.IntrinsicsDto;
import com.acme.camera.calibration.CalibrationConstraintChecker;
import com.acme.camera.calibration.CalibrationObservation;
import com.acme.camera.calibration.CalibrationProblem;
import com.acme.camera.calibration.CalibrationResult;
import com.acme.camera.calibration.CameraCalibrator;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
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
 * Orchestrates calibration jobs. Stateless: every request is validated from its own local
 * data and solved through an independent problem/result object, so concurrent calibration
 * jobs can never exchange parameters or error reports.
 *
 * <p>Validation is fail-fast and runs before any math: empty job, size-mismatched
 * observations, non-finite coordinates and structurally insufficient bundles are all
 * rejected with typed 422 errors.
 */
@Service
public class CalibrationJobService {

    private final CameraCalibrator calibrator;
    private final PinholeProjector projector;
    private final JobMetrics metrics;

    public CalibrationJobService(CameraCalibrator calibrator, PinholeProjector projector, JobMetrics metrics) {
        this.calibrator = calibrator;
        this.projector = projector;
        this.metrics = metrics;
    }

    public CalibrationJobResponse execute(CalibrationJobRequest request) {
        if (request == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD, "Missing request body");
        }
        InputValidator.requireImageSize(request.width(), request.height(), "");
        boolean estimateDistortion = request.estimateDistortion() == null
                || request.estimateDistortion();

        List<CalibrationObservationDto> observationDtos = request.observations();
        if (observationDtos == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing required field 'observations'", Map.of("field", "observations"));
        }
        if (observationDtos.isEmpty()) {
            throw new GeometryValidationException(ErrorType.EMPTY_OBSERVATION_SET,
                    "Calibration job carries zero observations; at least one view is required");
        }

        List<CalibrationObservation> observations = new ArrayList<>(observationDtos.size());
        for (int view = 0; view < observationDtos.size(); view++) {
            CalibrationObservationDto dto = observationDtos.get(view);
            String path = "observations[" + view + "]";
            if (dto == null) {
                throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                        "Missing observation at '" + path + "'", Map.of("field", path));
            }
            if (dto.worldPoints() == null || dto.pixels() == null) {
                throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                        "Observation '" + path + "' must carry both 'worldPoints' and 'pixels'",
                        Map.of("field", path));
            }
            if (dto.worldPoints().size() != dto.pixels().size()) {
                throw new GeometryValidationException(ErrorType.OBSERVATION_SIZE_MISMATCH,
                        "Observation " + view + " has " + dto.worldPoints().size()
                                + " world points but " + dto.pixels().size()
                                + " pixels; the lists must be index-aligned and equal in length",
                        Map.of("observation", view,
                                "worldPointCount", dto.worldPoints().size(),
                                "pixelCount", dto.pixels().size()));
            }
            List<Point3D> worldPoints = new ArrayList<>(dto.worldPoints().size());
            List<Point2D> pixels = new ArrayList<>(dto.pixels().size());
            for (int i = 0; i < dto.worldPoints().size(); i++) {
                worldPoints.add(InputValidator.requireWorldPoint(
                        dto.worldPoints().get(i), path + ".worldPoints[" + i + "]"));
                pixels.add(InputValidator.requirePixel(
                        dto.pixels().get(i), path + ".pixels[" + i + "]"));
            }
            observations.add(new CalibrationObservation(worldPoints, pixels));
        }

        CalibrationProblem problem = new CalibrationProblem(
                observations, request.width(), request.height(), estimateDistortion);
        // Explicit sufficiency gate before the optimizer is ever constructed.
        CalibrationConstraintChecker.check(problem);

        CalibrationResult result = calibrator.calibrate(problem);

        List<CalibratedPoseDto> poseDtos = new ArrayList<>(observations.size());
        int totalPoints = 0;
        double globalMax = 0.0;
        double globalSum = 0.0;
        for (int view = 0; view < observations.size(); view++) {
            Extrinsics pose = result.poses().get(view);
            CalibrationObservation observation = observations.get(view);
            double viewMax = 0.0;
            double viewSum = 0.0;
            for (int i = 0; i < observation.pointCount(); i++) {
                Point3D camera = pose.transform(observation.worldPoints().get(i));
                ProjectionResult projected = projector.project(
                        camera, result.intrinsics(), result.distortion());
                double error = Math.hypot(
                        projected.u() - observation.pixels().get(i).x(),
                        projected.v() - observation.pixels().get(i).y());
                viewMax = Math.max(viewMax, error);
                viewSum += error;
            }
            double viewMean = viewSum / observation.pointCount();
            poseDtos.add(new CalibratedPoseDto(view,
                    new ExtrinsicsDto(pose.rotation(), pose.translation()), viewMax, viewMean));
            globalMax = Math.max(globalMax, viewMax);
            globalSum += viewSum;
            totalPoints += observation.pointCount();
        }

        Intrinsics intrinsics = result.intrinsics();
        ConvergenceDto convergence = new ConvergenceDto(
                result.convergence().converged(),
                result.convergence().stopReason().name(),
                result.convergence().iterations(),
                result.convergence().initialRmsError(),
                result.convergence().finalRmsError(),
                result.convergence().rmsThresholdPixels());

        metrics.calibrationJobCompleted();
        return new CalibrationJobResponse(
                CameraCalibrator.METHOD,
                new IntrinsicsDto(intrinsics.fx(), intrinsics.fy(), intrinsics.cx(), intrinsics.cy(),
                        intrinsics.width(), intrinsics.height()),
                new DistortionDto(result.distortion().k1(), result.distortion().k2(),
                        result.distortion().p1(), result.distortion().p2()),
                poseDtos,
                convergence,
                observations.size(),
                totalPoints,
                globalMax,
                globalSum / totalPoints);
    }
}
