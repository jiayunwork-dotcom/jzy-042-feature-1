package com.acme.camera.api.dto;

import java.util.List;

/**
 * One calibration observation: world-frame positions of the control points and the pixels at
 * which they were observed in this image. The two lists must be equal in length and
 * index-aligned ({@code worldPoints[i]} was seen at {@code pixels[i]}).
 */
public record CalibrationObservationDto(List<Point3DDto> worldPoints, List<Point2DDto> pixels) {
}
