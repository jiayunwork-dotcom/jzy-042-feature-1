package com.acme.camera.api;

import com.acme.camera.api.dto.CameraDto;
import com.acme.camera.api.dto.DistortionDto;
import com.acme.camera.api.dto.ExtrinsicsDto;
import com.acme.camera.api.dto.IntrinsicsDto;
import com.acme.camera.api.dto.MatchDto;
import com.acme.camera.api.dto.Point2DDto;
import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.ProjectionJobResponse;
import com.acme.camera.api.dto.TriangulationJobRequest;
import com.acme.camera.api.dto.TriangulationJobResponse;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.acme.camera.core.ProjectionResult;
import com.acme.camera.job.ProjectionJobService;
import com.acme.camera.job.TriangulationJobService;
import com.acme.camera.preset.DemoRig;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Hammers the service with many distinct jobs at once and verifies that every response
 * contains exactly the pixels / error reports of its own job — nothing leaks between
 * concurrently running jobs.
 */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
class ConcurrencyIsolationTest {

    private static final int PROJECTION_JOBS = 24;
    private static final int TRIANGULATION_JOBS = 12;

    @Autowired
    private TestRestTemplate rest;

    @Autowired
    private ProjectionJobService projectionJobService;

    @Autowired
    private TriangulationJobService triangulationJobService;

    @Autowired
    private PinholeProjector projector;

    @Autowired
    private ObjectMapper mapper;

    private record Outcome(String kind, int index, int status, JsonNode body) {
    }

    @Test
    void concurrentJobsNeverMixTheirResults() throws Exception {
        // --- Build distinct jobs. ---
        List<ProjectionJobRequest> projectionRequests = new ArrayList<>();
        List<JsonNode> expectedProjection = new ArrayList<>();
        for (int i = 0; i < PROJECTION_JOBS; i++) {
            double fx = 600.0 + 13.0 * i;
            IntrinsicsDto intrinsics = new IntrinsicsDto(fx, fx, 640.0, 360.0, 1280, 720);
            List<Point3DDto> points = List.of(
                    new Point3DDto(0.1 * (i + 1), -0.05 * i, 2.0 + 0.1 * i),
                    new Point3DDto(-0.2, 0.03 * i, 3.0),
                    new Point3DDto(0.4, 0.4, 1.5 + 0.2 * i));
            ProjectionJobRequest request = new ProjectionJobRequest(intrinsics, null, points);
            projectionRequests.add(request);
            ProjectionJobResponse expected = projectionJobService.execute(request);
            expectedProjection.add(mapper.valueToTree(expected));
        }

        List<TriangulationJobRequest> triangulationRequests = new ArrayList<>();
        List<JsonNode> expectedTriangulation = new ArrayList<>();
        for (int i = 0; i < TRIANGULATION_JOBS; i++) {
            double scale = 1.0 + 0.05 * i;
            List<MatchDto> matches = new ArrayList<>();
            for (Point3D corner : DemoRig.cubeCorners()) {
                Point3D scaled = new Point3D(corner.x() * scale, corner.y() * scale, corner.z() * scale);
                ProjectionResult p1 = projector.project(DemoRig.CAMERA1.transform(scaled),
                        DemoRig.INTRINSICS, DemoRig.DISTORTION);
                ProjectionResult p2 = projector.project(DemoRig.CAMERA2.transform(scaled),
                        DemoRig.INTRINSICS, DemoRig.DISTORTION);
                matches.add(new MatchDto(new Point2DDto(p1.u(), p1.v()), new Point2DDto(p2.u(), p2.v())));
            }
            TriangulationJobRequest request = new TriangulationJobRequest(
                    demoCamera(DemoRig.CAMERA1.rotation(), DemoRig.CAMERA1.translation()),
                    demoCamera(DemoRig.CAMERA2.rotation(), DemoRig.CAMERA2.translation()),
                    matches);
            triangulationRequests.add(request);
            TriangulationJobResponse expected = triangulationJobService.execute(request);
            expectedTriangulation.add(mapper.valueToTree(expected));
        }

        // Baseline for the aggregate counters (after the expected-value computations above).
        JsonNode statusBefore = rest.getForEntity("/api/status", JsonNode.class).getBody();
        long projectionBefore = statusBefore.get("jobsCompleted").get("projection").asLong();
        long triangulationBefore = statusBefore.get("jobsCompleted").get("triangulation").asLong();

        // --- Fire everything concurrently. ---
        List<Callable<Outcome>> tasks = new ArrayList<>();
        for (int i = 0; i < PROJECTION_JOBS; i++) {
            final int index = i;
            tasks.add(() -> {
                ResponseEntity<JsonNode> response = rest.postForEntity(
                        "/api/jobs/projection", projectionRequests.get(index), JsonNode.class);
                return new Outcome("projection", index, response.getStatusCode().value(), response.getBody());
            });
        }
        for (int i = 0; i < TRIANGULATION_JOBS; i++) {
            final int index = i;
            tasks.add(() -> {
                ResponseEntity<JsonNode> response = rest.postForEntity(
                        "/api/jobs/triangulation", triangulationRequests.get(index), JsonNode.class);
                return new Outcome("triangulation", index, response.getStatusCode().value(), response.getBody());
            });
        }

        ExecutorService pool = Executors.newFixedThreadPool(16);
        List<Future<Outcome>> futures = pool.invokeAll(tasks);
        pool.shutdown();

        // --- Every response must match its own job's expected result exactly. ---
        for (Future<Outcome> future : futures) {
            Outcome outcome = future.get();
            assertEquals(HttpStatus.OK.value(), outcome.status(),
                    outcome.kind() + " job " + outcome.index() + " must succeed");
            JsonNode expected = outcome.kind().equals("projection")
                    ? expectedProjection.get(outcome.index())
                    : expectedTriangulation.get(outcome.index());
            assertEquals(expected, outcome.body(),
                    outcome.kind() + " job " + outcome.index() + " returned another job's data");
        }

        // --- Aggregate counters must reflect exactly this storm, nothing more. ---
        JsonNode statusAfter = rest.getForEntity("/api/status", JsonNode.class).getBody();
        assertEquals(projectionBefore + PROJECTION_JOBS,
                statusAfter.get("jobsCompleted").get("projection").asLong());
        assertEquals(triangulationBefore + TRIANGULATION_JOBS,
                statusAfter.get("jobsCompleted").get("triangulation").asLong());
    }

    private static CameraDto demoCamera(double[][] rotation, double[] translation) {
        return new CameraDto(
                new IntrinsicsDto(DemoRig.INTRINSICS.fx(), DemoRig.INTRINSICS.fy(),
                        DemoRig.INTRINSICS.cx(), DemoRig.INTRINSICS.cy(),
                        DemoRig.INTRINSICS.width(), DemoRig.INTRINSICS.height()),
                new DistortionDto(DemoRig.DISTORTION.k1(), DemoRig.DISTORTION.k2(),
                        DemoRig.DISTORTION.p1(), DemoRig.DISTORTION.p2()),
                new ExtrinsicsDto(rotation, translation));
    }
}
