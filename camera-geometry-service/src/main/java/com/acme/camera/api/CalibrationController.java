package com.acme.camera.api;

import com.acme.camera.api.dto.CalibrationJobRequest;
import com.acme.camera.api.dto.CalibrationJobResponse;
import com.acme.camera.job.CalibrationJobService;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** Calibration route: control-point observations in, camera parameters + poses + errors out. */
@RestController
@RequestMapping("/api")
public class CalibrationController {

    private final CalibrationJobService calibrationJobService;

    public CalibrationController(CalibrationJobService calibrationJobService) {
        this.calibrationJobService = calibrationJobService;
    }

    /** Submit a calibration job (linear initialization + iterative reprojection refinement). */
    @PostMapping("/jobs/calibration")
    public CalibrationJobResponse runCalibrationJob(@RequestBody CalibrationJobRequest request) {
        return calibrationJobService.execute(request);
    }
}
