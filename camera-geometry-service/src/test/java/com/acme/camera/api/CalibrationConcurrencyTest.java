package com.acme.camera.api;

import com.acme.camera.api.dto.CalibrationJobRequest;
import com.acme.camera.api.dto.CalibrationJobResponse;
import com.acme.camera.calibration.CalibrationFixtures;
import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.client.TestRestTemplate;
import org.springframework.http.ResponseEntity;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Fires many distinct calibration jobs simultaneously and verifies each response carries
 * exactly its own ground-truth intrinsics/distortion and its own error report — parameters
 * and per-job state never leak between concurrently running calibrations.
 */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
class CalibrationConcurrencyTest {

    private static final int JOBS = 12;

    @Autowired
    private TestRestTemplate rest;

    @Autowired
    private PinholeProjector projector;

    @Autowired
    private com.acme.camera.job.CalibrationJobService service;

    @Autowired
    private ObjectMapper mapper;

    private record Expected(double fx, double fy, double cx, double cy,
                            double k1, double k2, double p1, double p2) {
    }

    @Test
    void concurrentCalibrationJobsNeverMixTheirParametersOrErrorReports() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);

        List<CalibrationJobRequest> requests = new ArrayList<>();
        List<Expected> expected = new ArrayList<>();
        List<JsonNode> expectedBodies = new ArrayList<>();

        for (int i = 0; i < JOBS; i++) {
            // Every job uses a clearly different ground-truth camera so any mixing is detectable.
            Intrinsics intrinsics = new Intrinsics(
                    700.0 + 20.0 * i, 705.0 + 20.0 * i,
                    620.0 + 3.0 * i, 340.0 + 2.0 * i, 1280, 720);
            DistortionCoefficients distortion = new DistortionCoefficients(
                    -0.05 - 0.003 * i, 0.01 + 0.0008 * i,
                    0.0006 + 0.00005 * i, -0.001 - 0.0001 * i);
            var observations = CalibrationFixtures.observations(
                    projector, intrinsics, distortion, board, CalibrationFixtures.boardPoses());
            CalibrationJobRequest request = new CalibrationJobRequest(
                    CalibrationFixtures.observationDtos(observations), 1280, 720, true);
            requests.add(request);
            expected.add(new Expected(intrinsics.fx(), intrinsics.fy(), intrinsics.cx(), intrinsics.cy(),
                    distortion.k1(), distortion.k2(), distortion.p1(), distortion.p2()));
            CalibrationJobResponse response = service.execute(request);
            expectedBodies.add(mapper.valueToTree(response));
        }

        JsonNode statusBefore = rest.getForEntity("/api/status", JsonNode.class).getBody();
        long before = statusBefore.get("jobsCompleted").get("calibration").asLong();

        List<Callable<Integer>> tasks = new ArrayList<>();
        for (int i = 0; i < JOBS; i++) {
            final int index = i;
            tasks.add(() -> {
                ResponseEntity<JsonNode> response = rest.postForEntity(
                        "/api/jobs/calibration", requests.get(index), JsonNode.class);
        assertEquals(200, response.getStatusCode().value());
                assertBodyMatchesJob(response.getBody(), expected.get(index));
                // And it must match the serial execution of exactly this request.
        assertEquals(expectedBodies.get(index), response.getBody(),
                        "calibration job " + index + " returned another job's parameters or errors");
                return index;
            });
        }

        ExecutorService pool = Executors.newFixedThreadPool(8);
        List<Future<Integer>> futures = pool.invokeAll(tasks);
        pool.shutdown();
        for (Future<Integer> future : futures) {
            future.get();
        }

        JsonNode statusAfter = rest.getForEntity("/api/status", JsonNode.class).getBody();
        assertEquals(before + JOBS, statusAfter.get("jobsCompleted").get("calibration").asLong(),
                "exactly this storm must be counted, no more and no less");
    }

    private static void assertBodyMatchesJob(JsonNode body, Expected e) {
        JsonNode k = body.get("intrinsics");
        assertEquals(e.fx(), k.get("fx").asDouble(), 1e-4);
        assertEquals(e.fy(), k.get("fy").asDouble(), 1e-4);
        assertEquals(e.cx(), k.get("cx").asDouble(), 1e-4);
        assertEquals(e.cy(), k.get("cy").asDouble(), 1e-4);
        JsonNode d = body.get("distortion");
        assertEquals(e.k1(), d.get("k1").asDouble(), 1e-4);
        assertEquals(e.k2(), d.get("k2").asDouble(), 1e-4);
        assertEquals(e.p1(), d.get("p1").asDouble(), 1e-5);
        assertEquals(e.p2(), d.get("p2").asDouble(), 1e-5);
        assertTrue(body.get("convergence").get("converged").asBoolean());
        assertTrue(body.get("maxReprojectionError").asDouble() < 1e-6);
        assertTrue(body.get("meanReprojectionError").asDouble() < 1e-6);
        assertEquals(5, body.get("poses").size());
    }
}
