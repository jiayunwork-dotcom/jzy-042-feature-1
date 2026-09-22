package com.acme.camera.api;

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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

/** API-level tests for the triangulation job endpoint. */
@SpringBootTest
@AutoConfigureMockMvc
class TriangulationApiTest {

    @Autowired
    private MockMvc mvc;

    @Autowired
    private ObjectMapper mapper;

    private JsonNode demoTriangulationJob() throws Exception {
        MvcResult result = mvc.perform(get("/api/presets")).andExpect(status().isOk()).andReturn();
        return mapper.readTree(result.getResponse().getContentAsString())
                .get("cubeDemo").get("triangulationJob");
    }

    @Test
    void cubeDemoTriangulationReprojectsBackOntoOriginalPixels() throws Exception {
        JsonNode job = demoTriangulationJob();
        MvcResult result = mvc.perform(post("/api/jobs/triangulation")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(job)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.triangulationMethod").value("DLT"))
                .andExpect(jsonPath("$.pairCount").value(8))
                .andExpect(jsonPath("$.validPairCount").value(8))
                .andReturn();

        JsonNode response = mapper.readTree(result.getResponse().getContentAsString());
        double maxError = response.get("maxReprojectionError").asDouble();
        double meanError = response.get("meanReprojectionError").asDouble();
        assertTrue(maxError < 1e-6, "max reprojection error must be sub-micro-pixel, got " + maxError);
        assertTrue(meanError < 1e-6, "mean reprojection error must be sub-micro-pixel, got " + meanError);

        // Every recovered 3D point must match the known cube corner.
        JsonNode corners = mapper.readTree(
                        mvc.perform(get("/api/presets")).andReturn().getResponse().getContentAsString())
                .get("cubeDemo").get("worldCorners");
        JsonNode pairs = response.get("pairs");
        for (int i = 0; i < 8; i++) {
            JsonNode point = pairs.get(i).get("point3d");
            JsonNode corner = corners.get(i);
            assertEquals(corner.get("x").asDouble(), point.get("x").asDouble(), 1e-6, "corner " + i);
            assertEquals(corner.get("y").asDouble(), point.get("y").asDouble(), 1e-6, "corner " + i);
            assertEquals(corner.get("z").asDouble(), point.get("z").asDouble(), 1e-6, "corner " + i);
            assertTrue(pairs.get(i).get("valid").asBoolean());
        }
    }

    @Test
    void missingExtrinsicsOnOneCameraIsRejected() throws Exception {
        ObjectNode job = (ObjectNode) demoTriangulationJob();
        ((ObjectNode) job.get("camera2")).remove("extrinsics");

        mvc.perform(post("/api/jobs/triangulation")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(job)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("MISSING_EXTRINSICS"))
                .andExpect(jsonPath("$.error.details.field").value("camera2.extrinsics"));
    }

    @Test
    void zeroMatchedPairsAreRejected() throws Exception {
        ObjectNode job = (ObjectNode) demoTriangulationJob();
        job.putArray("matches");

        mvc.perform(post("/api/jobs/triangulation")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(job)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("EMPTY_MATCH_SET"));
    }

    @Test
    void missingCameraIsRejected() throws Exception {
        ObjectNode job = (ObjectNode) demoTriangulationJob();
        job.remove("camera1");

        mvc.perform(post("/api/jobs/triangulation")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(mapper.writeValueAsString(job)))
                .andExpect(status().isUnprocessableEntity())
                .andExpect(jsonPath("$.error.type").value("MISSING_FIELD"));
    }
}
