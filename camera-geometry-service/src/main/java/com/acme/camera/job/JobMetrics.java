package com.acme.camera.job;

import org.springframework.stereotype.Component;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Process-wide, thread-safe job counters. Aggregate statistics only — no per-job data ever
 * lives here, which is what keeps concurrently running jobs isolated from each other.
 */
@Component
public class JobMetrics {

    private final Instant startedAt = Instant.now();
    private final AtomicLong projectionJobsCompleted = new AtomicLong();
    private final AtomicLong triangulationJobsCompleted = new AtomicLong();

    public Instant startedAt() {
        return startedAt;
    }

    public long projectionJobsCompleted() {
        return projectionJobsCompleted.get();
    }

    public long triangulationJobsCompleted() {
        return triangulationJobsCompleted.get();
    }

    public void projectionJobCompleted() {
        projectionJobsCompleted.incrementAndGet();
    }

    public void triangulationJobCompleted() {
        triangulationJobsCompleted.incrementAndGet();
    }
}
