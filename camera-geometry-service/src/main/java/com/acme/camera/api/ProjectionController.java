package com.acme.camera.api;

import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.ProjectionJobResponse;
import com.acme.camera.api.dto.SingleProjectionRequest;
import com.acme.camera.api.dto.SingleProjectionResponse;
import com.acme.camera.core.ProjectionResult;
import com.acme.camera.job.ProjectionJobService;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** Projection routes: single-point projection and batch projection jobs. */
@RestController
@RequestMapping("/api")
public class ProjectionController {

    private final ProjectionJobService projectionJobService;

    public ProjectionController(ProjectionJobService projectionJobService) {
        this.projectionJobService = projectionJobService;
    }

    /** Project one camera-frame 3D point. Same projection pipeline as the batch job. */
    @PostMapping("/project")
    public SingleProjectionResponse projectSingle(@RequestBody SingleProjectionRequest request) {
        ProjectionResult result = projectionJobService.projectSingle(request);
        return new SingleProjectionResponse(result.u(), result.v(), result.inBounds());
    }

    /** Submit a batch projection job. */
    @PostMapping("/jobs/projection")
    public ProjectionJobResponse runProjectionJob(@RequestBody ProjectionJobRequest request) {
        return projectionJobService.execute(request);
    }
}
