package com.acme.camera.core;

/**
 * Result of projecting one 3D point.
 *
 * @param u        pixel column
 * @param v        pixel row
 * @param inBounds true when the pixel falls inside the image: 0 &lt;= u &lt; width and 0 &lt;= v &lt; height
 */
public record ProjectionResult(double u, double v, boolean inBounds) {
}
