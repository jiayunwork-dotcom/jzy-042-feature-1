package com.acme.camera.api.dto;

import com.acme.camera.core.Point3D;

/**
 * One triangulated matched pair.
 *
 * @param valid false when the triangulated point falls at/behind one of the cameras and
 *              therefore cannot be reprojected; error fields are null in that case
 */
public record TriangulatedPair(int index,
                               Point3D point3d,
                               boolean valid,
                               Double reprojectionErrorView1,
                               Double reprojectionErrorView2,
                               Double reprojectionError) {
}
