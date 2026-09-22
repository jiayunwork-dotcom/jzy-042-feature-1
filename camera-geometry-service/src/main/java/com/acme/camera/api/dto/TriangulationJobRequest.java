package com.acme.camera.api.dto;

import java.util.List;

/** Triangulation job: two fully specified cameras plus matched two-view pixel pairs. */
public record TriangulationJobRequest(CameraDto camera1, CameraDto camera2, List<MatchDto> matches) {
}
