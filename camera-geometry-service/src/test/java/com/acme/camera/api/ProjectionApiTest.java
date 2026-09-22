package com.acme.camera.api;

import com.acme.camera.api.dto.DistortionDto;
import com.acme.camera.api.dto.IntrinsicsDto;
import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.SingleProjectionRequest;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.MvcResult;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

/** API-level tests for the projection endpoints: validation, consistency, demo fixture. */
@SpringBootTest
@AutoConfigureMockMvc
class ProjectionApiTest {

    @Autowired
    private MockMvc mvc;

    @Autowired
    private ObjectMapper mapper;

    private static final IntrinsicsDto HD = new IntrinsicsDto(800.0, 800.0, 640.0, 360.0, 1280, 720);
    private static final DistortionDto ZERO_DISTORTION = new DistortionDto(0.0, 0.0, 0.0, 0.0);

    private String postJson(String url, Object body) throws Exception {
        return mvc.perform(post(url).contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(body)))
                .andExpect(status().isOk())
                .andReturn().getResponse().getContentAsString();
    }

    @Test
    void zeroDistortionCoefficientsProduceSamePixelsAsNoDistortion() throws Exception {
        List<Point3DDto> points = List.of(
                new Point3DDto(0.3, -0.2, 2.0),
                new Point3DDto(-1.1, 0.4, 3.5),
                new Point3DDto(0.0, 0.0, 1.0));

        String explicitZeros = postJson("/api/jobs/projection",
                new ProjectionJobRequest(HD, ZERO_DISTORTION, points));
        String omitted = postJson("/api/jobs/projection",
                new ProjectionJobRequest(HD, null, points));

        assertEquals(mapper.readTree(omitted), mapper.readTree(explicitZeros),
                "k1=k2=p1=p2=0 must be pixel-identical to no distortion at all");
    }

    @Test
    void pointBehindCameraIsRejectedBeforeComputation() throws Exception {
        SingleProjectionRequest behind = new SingleProjectionRequest(HD, null, new Point3DDto(0.5, 0.5, -1.0));
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(behind)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("POINT_BEHIND_CAMERA"));

        SingleProjectionRequest atZero = new SingleProjectionRequest(HD, null, new Point3DDto(0.5, 0.5, 0.0));
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(atZero)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("POINT_BEHIND_CAMERA"));
    }

    @Test
    void oneBadPointRejectsTheWholeBatchJob() throws Exception {
        ProjectionJobRequest job = new ProjectionJobRequest(HD, null, List.of(
                new Point3DDto(0.1, 0.1, 2.0),
                new Point3DDto(0.2, 0.2, 3.0),
                new Point3DDto(0.0, 0.0, -0.5)));
        mvc.perform(post("/api/jobs/projection").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(job)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("POINT_BEHIND_CAMERA"))
                .andExpect(jsonPath("$.error.details.point").value("points[2]"));
    }

    @Test
    void nonPositiveFocalLengthIsRejected() throws Exception {
        IntrinsicsDto zeroFx = new IntrinsicsDto(0.0, 800.0, 640.0, 360.0, 1280, 720);
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(
                                new SingleProjectionRequest(zeroFx, null, new Point3DDto(0.1, 0.1, 2.0)))))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_FOCAL_LENGTH"));

        IntrinsicsDto negativeFy = new IntrinsicsDto(800.0, -3.0, 640.0, 360.0, 1280, 720);
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(
                                new SingleProjectionRequest(negativeFy, null, new Point3DDto(0.1, 0.1, 2.0)))))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_FOCAL_LENGTH"));
    }

    @Test
    void zeroImageDimensionIsRejected() throws Exception {
        IntrinsicsDto zeroWidth = new IntrinsicsDto(800.0, 800.0, 640.0, 360.0, 0, 720);
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(
                                new SingleProjectionRequest(zeroWidth, null, new Point3DDto(0.1, 0.1, 2.0)))))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("INVALID_IMAGE_SIZE"));
    }

    @Test
    void missingIntrinsicsFieldIsRejected() throws Exception {
        String body = """
                {"intrinsics": {"fy": 800.0, "cx": 640.0, "cy": 360.0, "width": 1280, "height": 720},
                 "point": {"x": 0.1, "y": 0.1, "z": 2.0}}
                """;
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON).content(body))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("MISSING_INTRINSICS_FIELD"))
                .andExpect(jsonPath("$.error.details.field").value("intrinsics.fx"));
    }

    @Test
    void malformedJsonIsRejected() throws Exception {
        mvc.perform(post("/api/project").contentType(MediaType.APPLICATION_JSON)
                        .content("{not json"))
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.error.type").value("MALFORMED_REQUEST"));
    }

    @Test
    void singlePointAndBatchJobShareTheSameProjectionResult() throws Exception {
        DistortionDto distortion = new DistortionDto(-0.1, 0.01, 0.001, -0.002);
        Point3DDto point = new Point3DDto(0.35, -0.22, 2.4);

        String singleBody = postJson("/api/project", new SingleProjectionRequest(HD, distortion, point));
        JsonNode single = mapper.readTree(singleBody);

        String batchBody = postJson("/api/jobs/projection",
                new ProjectionJobRequest(HD, distortion, List.of(point)));
        JsonNode batchFirst = mapper.readTree(batchBody).get("results").get(0);

        assertEquals(single.get("u").asDouble(), batchFirst.get("u").asDouble(), 0.0,
                "single-point endpoint and batch job must agree bit-for-bit on the same point");
        assertEquals(single.get("v").asDouble(), batchFirst.get("v").asDouble(), 0.0);
        assertEquals(single.get("inBounds").asBoolean(), batchFirst.get("inBounds").asBoolean());
    }

    @Test
    void cubeDemoProjectionJobLandsEntirelyInsideTheImage() throws Exception {
        MvcResult presetsResult = mvc.perform(get("/api/presets"))
                .andExpect(status().isOk())
                .andReturn();
        JsonNode projectionJob = mapper.readTree(presetsResult.getResponse().getContentAsString())
                .get("cubeDemo").get("projectionJob");

        MvcResult jobResult = mvc.perform(post("/api/jobs/projection")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(projectionJob)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.totalPoints").value(8))
                .andExpect(jsonPath("$.outOfBoundsCount").value(0))
                .andReturn();
        JsonNode results = mapper.readTree(jobResult.getResponse().getContentAsString()).get("results");
        for (JsonNode point : results) {
            assertTrue(point.get("inBounds").asBoolean(), "every cube corner must be in bounds");
        }
    }
}
