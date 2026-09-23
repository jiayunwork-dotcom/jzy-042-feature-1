package com.acme.camera.api;

import com.acme.camera.job.JobMetrics;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.time.Duration;
import java.time.Instant;

/** Liveness/status route. */
@RestController
@RequestMapping("/api")
public class StatusController {

    public static final String VERSION = "1.0.0";

    private final JobMetrics metrics;

    public StatusController(JobMetrics metrics) {
        this.metrics = metrics;
    }

    @GetMapping("/status")
    public StatusResponse status() {
        return new StatusResponse(
                "UP",
                "camera-geometry-service",
                VERSION,
                metrics.startedAt(),
                Duration.between(metrics.startedAt(), Instant.now()).toSeconds(),
                new JobCounts(metrics.projectionJobsCompleted(),
                        metrics.triangulationJobsCompleted(),
                        metrics.calibrationJobsCompleted()));
    }

    public record StatusResponse(String status,
                                 String service,
                                 String version,
                                 Instant startedAt,
                                 long uptimeSeconds,
                                 JobCounts jobsCompleted) {
    }

    public record JobCounts(long projection, long triangulation, long calibration) {
    }
}
