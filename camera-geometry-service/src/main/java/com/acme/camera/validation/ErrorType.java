package com.acme.camera.validation;

/** Stable, machine-readable error categories returned in every structured error body. */
public enum ErrorType {
    /** Request body is not parseable JSON / wrong shape. */
    MALFORMED_REQUEST,
    /** A required field is absent (details carry the field path). */
    MISSING_FIELD,
    /** A field is present but not a finite number where one is required. */
    INVALID_VALUE,
    /** An intrinsics field (fx, fy, cx, cy, width, height) is absent. */
    MISSING_INTRINSICS_FIELD,
    /** A distortion field (k1, k2, p1, p2) is absent while a distortion object was supplied. */
    MISSING_DISTORTION_FIELD,
    /** fx or fy is not a positive finite number. */
    INVALID_FOCAL_LENGTH,
    /** Image width or height is not a positive integer. */
    INVALID_IMAGE_SIZE,
    /** A 3D point has Z &lt;= 0 in camera coordinates and cannot be projected. */
    POINT_BEHIND_CAMERA,
    /** A triangulation camera is missing its extrinsics block. */
    MISSING_EXTRINSICS,
    /** Extrinsics are present but malformed (rotation not 3x3 / translation not 3 / non-finite). */
    INVALID_EXTRINSICS,
    /** A triangulation job carries zero matched pairs. */
    EMPTY_MATCH_SET,
    /** A calibration job carries zero observations. */
    EMPTY_OBSERVATION_SET,
    /** One calibration observation has mismatched world-point / pixel counts. */
    OBSERVATION_SIZE_MISMATCH,
    /** Calibration observations cannot determine the unknowns (too few/degenerate). */
    INSUFFICIENT_CONSTRAINTS,
    /** Unanticipated server-side failure. */
    INTERNAL_ERROR
}
