package com.acme.camera.validation;

import com.acme.camera.api.dto.DistortionDto;
import com.acme.camera.api.dto.ExtrinsicsDto;
import com.acme.camera.api.dto.IntrinsicsDto;
import com.acme.camera.api.dto.Point2DDto;
import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.core.DistortionCoefficients;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.Point2D;
import com.acme.camera.core.Point3D;

import java.util.Map;

/**
 * Boundary between untrusted wire DTOs and the typed core model. Every check runs BEFORE
 * any projection/triangulation math, so illegal input never reaches the geometry layer.
 * All failures are typed {@link GeometryValidationException}s.
 */
public final class InputValidator {

    private InputValidator() {
    }

    public static Intrinsics requireIntrinsics(IntrinsicsDto dto, String path) {
        if (dto == null) {
            throw new GeometryValidationException(ErrorType.MISSING_INTRINSICS_FIELD,
                    "Missing intrinsics object at '" + path + "'", Map.of("field", path));
        }
        requirePresent(dto.fx(), path + ".fx", ErrorType.MISSING_INTRINSICS_FIELD);
        requirePresent(dto.fy(), path + ".fy", ErrorType.MISSING_INTRINSICS_FIELD);
        requirePresent(dto.cx(), path + ".cx", ErrorType.MISSING_INTRINSICS_FIELD);
        requirePresent(dto.cy(), path + ".cy", ErrorType.MISSING_INTRINSICS_FIELD);
        requirePresent(dto.width(), path + ".width", ErrorType.MISSING_INTRINSICS_FIELD);
        requirePresent(dto.height(), path + ".height", ErrorType.MISSING_INTRINSICS_FIELD);

        if (!Double.isFinite(dto.fx()) || dto.fx() <= 0.0) {
            throw new GeometryValidationException(ErrorType.INVALID_FOCAL_LENGTH,
                    "fx must be a positive finite number, got " + dto.fx(),
                    Map.of("field", path + ".fx", "value", dto.fx()));
        }
        if (!Double.isFinite(dto.fy()) || dto.fy() <= 0.0) {
            throw new GeometryValidationException(ErrorType.INVALID_FOCAL_LENGTH,
                    "fy must be a positive finite number, got " + dto.fy(),
                    Map.of("field", path + ".fy", "value", dto.fy()));
        }
        if (!Double.isFinite(dto.cx()) || !Double.isFinite(dto.cy())) {
            throw new GeometryValidationException(ErrorType.INVALID_VALUE,
                    "Principal point (cx, cy) must be finite", Map.of("field", path));
        }
        if (dto.width() <= 0 || dto.height() <= 0) {
            throw new GeometryValidationException(ErrorType.INVALID_IMAGE_SIZE,
                    "Image width and height must be positive, got " + dto.width() + "x" + dto.height(),
                    Map.of("width", dto.width(), "height", dto.height()));
        }
        return new Intrinsics(dto.fx(), dto.fy(), dto.cx(), dto.cy(), dto.width(), dto.height());
    }

    /** Null distortion object means "no distortion"; a present object must be fully specified. */
    public static DistortionCoefficients requireDistortion(DistortionDto dto, String path) {
        if (dto == null) {
            return DistortionCoefficients.ZERO;
        }
        requirePresent(dto.k1(), path + ".k1", ErrorType.MISSING_DISTORTION_FIELD);
        requirePresent(dto.k2(), path + ".k2", ErrorType.MISSING_DISTORTION_FIELD);
        requirePresent(dto.p1(), path + ".p1", ErrorType.MISSING_DISTORTION_FIELD);
        requirePresent(dto.p2(), path + ".p2", ErrorType.MISSING_DISTORTION_FIELD);
        if (!Double.isFinite(dto.k1()) || !Double.isFinite(dto.k2())
                || !Double.isFinite(dto.p1()) || !Double.isFinite(dto.p2())) {
            throw new GeometryValidationException(ErrorType.INVALID_VALUE,
                    "Distortion coefficients must be finite", Map.of("field", path));
        }
        return new DistortionCoefficients(dto.k1(), dto.k2(), dto.p1(), dto.p2());
    }

    public static Extrinsics requireExtrinsics(ExtrinsicsDto dto, String path) {
        if (dto == null) {
            throw new GeometryValidationException(ErrorType.MISSING_EXTRINSICS,
                    "Missing extrinsics (rotation + translation) at '" + path + "'",
                    Map.of("field", path));
        }
        double[][] r = dto.rotation();
        double[] t = dto.translation();
        if (r == null || r.length != 3 || r[0] == null || r[1] == null || r[2] == null
                || r[0].length != 3 || r[1].length != 3 || r[2].length != 3) {
            throw new GeometryValidationException(ErrorType.INVALID_EXTRINSICS,
                    "Rotation must be a 3x3 matrix at '" + path + ".rotation'",
                    Map.of("field", path + ".rotation"));
        }
        if (t == null || t.length != 3) {
            throw new GeometryValidationException(ErrorType.INVALID_EXTRINSICS,
                    "Translation must be a 3-vector at '" + path + ".translation'",
                    Map.of("field", path + ".translation"));
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (!Double.isFinite(r[i][j])) {
                    throw new GeometryValidationException(ErrorType.INVALID_EXTRINSICS,
                            "Rotation contains a non-finite value at [" + i + "][" + j + "]",
                            Map.of("field", path + ".rotation"));
                }
            }
            if (!Double.isFinite(t[i])) {
                throw new GeometryValidationException(ErrorType.INVALID_EXTRINSICS,
                        "Translation contains a non-finite value at index " + i,
                        Map.of("field", path + ".translation"));
            }
        }
        return new Extrinsics(r, t);
    }

    /** A camera-frame 3D point that is legal to project: finite, and strictly in front of the camera. */
    public static Point3D requireProjectablePoint(Point3DDto dto, String path) {
        if (dto == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing point at '" + path + "'", Map.of("field", path));
        }
        requirePresent(dto.x(), path + ".x", ErrorType.MISSING_FIELD);
        requirePresent(dto.y(), path + ".y", ErrorType.MISSING_FIELD);
        requirePresent(dto.z(), path + ".z", ErrorType.MISSING_FIELD);
        if (!Double.isFinite(dto.x()) || !Double.isFinite(dto.y()) || !Double.isFinite(dto.z())) {
            throw new GeometryValidationException(ErrorType.INVALID_VALUE,
                    "Point coordinates must be finite at '" + path + "'", Map.of("field", path));
        }
        if (dto.z() <= 0.0) {
            throw new GeometryValidationException(ErrorType.POINT_BEHIND_CAMERA,
                    "Point at '" + path + "' has Z=" + dto.z()
                            + "; only points in front of the camera (Z > 0) can be projected",
                    Map.of("point", path, "z", dto.z()));
        }
        return new Point3D(dto.x(), dto.y(), dto.z());
    }

    /** A matched pixel: must be present and finite (it may lie outside the image bounds). */
    public static Point2D requirePixel(Point2DDto dto, String path) {
        if (dto == null) {
            throw new GeometryValidationException(ErrorType.MISSING_FIELD,
                    "Missing pixel at '" + path + "'", Map.of("field", path));
        }
        requirePresent(dto.x(), path + ".x", ErrorType.MISSING_FIELD);
        requirePresent(dto.y(), path + ".y", ErrorType.MISSING_FIELD);
        if (!Double.isFinite(dto.x()) || !Double.isFinite(dto.y())) {
            throw new GeometryValidationException(ErrorType.INVALID_VALUE,
                    "Pixel coordinates must be finite at '" + path + "'", Map.of("field", path));
        }
        return new Point2D(dto.x(), dto.y());
    }

    private static void requirePresent(Object value, String field, ErrorType type) {
        if (value == null) {
            throw new GeometryValidationException(type,
                    "Missing required field '" + field + "'", Map.of("field", field));
        }
    }
}
