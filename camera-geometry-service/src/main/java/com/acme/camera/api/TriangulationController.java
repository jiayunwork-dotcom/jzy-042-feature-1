package com.acme.camera.api;

import com.acme.camera.api.dto.TriangulationJobRequest;
import com.acme.camera.api.dto.TriangulationJobResponse;
import com.acme.camera.job.TriangulationJobService;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** Triangulation route: two-view matched pairs in, 3D points + reprojection errors out. */
@RestController
@RequestMapping("/api")
public class TriangulationController {

    private final TriangulationJobService triangulationJobService;

    public TriangulationController(TriangulationJobService triangulationJobService) {
        this.triangulationJobService = triangulationJobService;
    }

    /** Submit a triangulation job (DLT). */
    @PostMapping("/jobs/triangulation")
    public TriangulationJobResponse runTriangulationJob(@RequestBody TriangulationJobRequest request) {
        return triangulationJobService.execute(request);
    }
}
