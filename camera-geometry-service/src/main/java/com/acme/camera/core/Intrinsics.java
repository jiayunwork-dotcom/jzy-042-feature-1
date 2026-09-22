package com.acme.camera.core;

/**
 * Pinhole intrinsics plus the image plane they project onto.
 *
 * @param fx     focal length in x (pixels), must be &gt; 0
 * @param fy     focal length in y (pixels), must be &gt; 0
 * @param cx     principal point x (pixels)
 * @param cy     principal point y (pixels)
 * @param width  image width in pixels, must be &gt; 0
 * @param height image height in pixels, must be &gt; 0
 */
public record Intrinsics(double fx, double fy, double cx, double cy, int width, int height) {
}
