package com.acme.camera.api;

import com.acme.camera.api.dto.CalibrationJobRequest;
import com.acme.camera.api.dto.CalibrationObservationDto;
import com.acme.camera.calibration.CalibrationFixtures;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.MvcResult;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

/**
 * API-level tests for the calibration job endpoint: the four acceptance invariants as seen
 * over HTTP, plus fail-fast validation and the status counter.
 */
@SpringBootTest
@AutoConfigureMockMvc
class CalibrationApiTest {

    @Autowired
    private MockMvc mvc;

    @Autowired
    private ObjectMapper mapper;

    @Autowired
    private PinholeProjector projector;

    private JsonNode successfulBoardJob() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, CalibrationFixtures.boardPoses());
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 1280, 720, true);
        MvcResult result = mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isOk())
                .andReturn();
        return mapper.readTree(result.getResponse().getContentAsString());
    }

    // ── Invariant 1 + 2: round-trip with non-zero distortion over HTTP ──────

    @Test
    void nonZeroDistortionRoundTripRecoversParametersAndNearZeroErrors() throws Exception {
        JsonNode response = successfulBoardJob();

        assertEquals("BUNDLE_REPROJECTION_LM", response.get("calibrationMethod").asText());
        JsonNode k = response.get("intrinsics");
        assertEquals(830.0, k.get("fx").asDouble(), 1e-4);
        assertEquals(820.0, k.get("fy").asDouble(), 1e-4);
        assertEquals(646.0, k.get("cx").asDouble(), 1e-4);
        assertEquals(355.0, k.get("cy").asDouble(), 1e-4);

        JsonNode d = response.get("distortion");
        assertEquals(-0.08, d.get("k1").asDouble(), 1e-4);
        assertEquals(0.02, d.get("k2").asDouble(), 1e-4);
        assertEquals(0.001, d.get("p1").asDouble(), 1e-5);
        assertEquals(-0.002, d.get("p2").asDouble(), 1e-5);

        assertTrue(response.get("maxReprojectionError").asDouble() < 1e-6,
                "max reprojection error must be near zero");
        assertTrue(response.get("meanReprojectionError").asDouble() < 1e-6,
                "mean reprojection error must be near zero");

        JsonNode convergence = response.get("convergence");
        assertTrue(convergence.get("converged").asBoolean());
        assertEquals("RESIDUAL_THRESHOLD_REACHED", convergence.get("stopReason").asText());
        assertTrue(convergence.get("iterations").asInt() > 0);
        assertTrue(convergence.get("initialRmsError").asDouble()
                > convergence.get("finalRmsError").asDouble());
        assertTrue(convergence.get("finalRmsError").asDouble()
                <= convergence.get("rmsThresholdPixels").asDouble());

        assertEquals(5, response.get("observationCount").asInt());
        assertEquals(100, response.get("totalPoints").asInt());
        assertEquals(5, response.get("poses").size());
    }

    @Test
    void omittingEstimateDistortionDefaultsToEstimating() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, CalibrationFixtures.boardPoses());
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 1280, 720, null);
        MvcResult result = mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isOk())
                .andReturn();
        JsonNode response = mapper.readTree(result.getResponse().getContentAsString());
        assertEquals(-0.08, response.get("distortion").get("k1").asDouble(), 1e-4,
                "distortion must be estimated by default and recovered");
    }

    @Test
    void pinholeOnlyCalibrationReportsZeroDistortion() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses());
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 1280, 720, false);
        MvcResult result = mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isOk())
                .andReturn();
        JsonNode response = mapper.readTree(result.getResponse().getContentAsString());
        assertEquals(0.0, response.get("distortion").get("k1").asDouble(), 0.0);
        assertTrue(response.get("meanReprojectionError").asDouble() < 1e-6);
    }

    // ── Invariant 3: typed structural rejection over HTTP ───────────────────

    @Test
    void emptyObservationSetIsRejected() throws Exception {
        CalibrationJobRequest request = new CalibrationJobRequest(List.of(), 1280, 720, true);
        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("EMPTY_OBSERVATION_SET"));
    }

    @Test
    void mismatchedWorldAndPixelCountsAreRejected() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses().subList(0, 1));
        List<CalibrationObservationDto> dtos = new ArrayList<>(CalibrationFixtures.observationDtos(observations));
        // Drop one pixel from the single observation so the lengths disagree.
        CalibrationObservationDto original = dtos.get(0);
        dtos.set(0, new CalibrationObservationDto(
                original.worldPoints(), original.pixels().subList(0, original.pixels().size() - 1)));
        CalibrationJobRequest request = new CalibrationJobRequest(dtos, 1280, 720, true);

        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("OBSERVATION_SIZE_MISMATCH"))
                .andExpect(jsonPath("$.error.details.observation").value(0));
    }

    @Test
    void nonFiniteWorldCoordinateIsRejectedBeforeComputation() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses());
        List<CalibrationObservationDto> dtos = CalibrationFixtures.observationDtos(observations);
        ObjectNode body = (ObjectNode) mapper.valueToTree(
                new CalibrationJobRequest(dtos, 1280, 720, true));
        ((ObjectNode) body.withArray("observations").get(0).withArray("worldPoints").get(3))
                .put("z", Double.NaN);

        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(body)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_VALUE"));
    }

    @Test
    void nonFinitePixelIsRejectedBeforeComputation() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses());
        List<CalibrationObservationDto> dtos = CalibrationFixtures.observationDtos(observations);
        ObjectNode body = (ObjectNode) mapper.valueToTree(
                new CalibrationJobRequest(dtos, 1280, 720, true));
        ((ObjectNode) body.withArray("observations").get(1).withArray("pixels").get(0))
                .put("x", Double.POSITIVE_INFINITY);

        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(body)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_VALUE"));
    }

    @Test
    void singlePlanarViewIsRejectedAsUnderConstrained() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.DISTORTION,
                board, CalibrationFixtures.boardPoses().subList(0, 1));
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 1280, 720, true);

        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INSUFFICIENT_CONSTRAINTS"))
                .andExpect(jsonPath("$.error.details.requiredViews").value(2));
    }

    @Test
    void tooFewEquationsForTheUnknownCountAreRejected() throws Exception {
        // Two planar views of four points each with distortion: 16 equations vs 20 unknowns
        // (8 global + 2 × 6 poses). Enough views to pass the planar ambiguity gate, still
        // under-determined in the raw equation count.
        List<Point3D> fourPoints = CalibrationFixtures.boardPoints(2, 2, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                fourPoints, CalibrationFixtures.boardPoses().subList(0, 2));
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 1280, 720, true);

        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INSUFFICIENT_CONSTRAINTS"))
                .andExpect(jsonPath("$.error.details.unknowns").value(20))
                .andExpect(jsonPath("$.error.details.equations").value(16));
    }

    @Test
    void missingObservationsFieldIsRejected() throws Exception {
        String body = "{\"width\":1280,\"height\":720,\"estimateDistortion\":true}";
        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON).content(body))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("MISSING_FIELD"));
    }

    @Test
    void nonPositiveImageSizeIsRejected() throws Exception {
        List<Point3D> board = CalibrationFixtures.boardPoints(4, 5, 0.3);
        var observations = CalibrationFixtures.observations(
                projector, CalibrationFixtures.INTRINSICS, CalibrationFixtures.ZERO_DISTORTION,
                board, CalibrationFixtures.boardPoses());
        CalibrationJobRequest request = new CalibrationJobRequest(
                CalibrationFixtures.observationDtos(observations), 0, 720, true);
        mvc.perform(post("/api/jobs/calibration")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(request)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_IMAGE_SIZE"));
    }

    @Test
    void malformedJsonIsRejected() throws Exception {
        mvc.perform(post("/api/jobs/calibration").contentType(MediaType.APPLICATION_JSON)
                        .content("{not json"))
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.error.type").value("MALFORMED_REQUEST"));
    }

    // ── status counter includes calibration jobs ─────────────────────────────

    @Test
    void statusCounterIncrementsWithCalibrationJobs() throws Exception {
        JsonNode before = mapper.readTree(
                mvc.perform(get("/api/status")).andReturn().getResponse().getContentAsString());
        long beforeCount = before.get("jobsCompleted").get("calibration").asLong();

        successfulBoardJob();

        JsonNode after = mapper.readTree(
                mvc.perform(get("/api/status")).andReturn().getResponse().getContentAsString());
        assertEquals(beforeCount + 1, after.get("jobsCompleted").get("calibration").asLong());
        // Existing counters keep their place and name.
        assertTrue(after.get("jobsCompleted").has("projection"));
        assertTrue(after.get("jobsCompleted").has("triangulation"));
    }
}
