package com.acme.camera.preset;

import com.acme.camera.api.dto.CameraDto;
import com.acme.camera.api.dto.DistortionDto;
import com.acme.camera.api.dto.ExtrinsicsDto;
import com.acme.camera.api.dto.IntrinsicsDto;
import com.acme.camera.api.dto.MatchDto;
import com.acme.camera.api.dto.Point2DDto;
import com.acme.camera.api.dto.Point3DDto;
import com.acme.camera.api.dto.ProjectionJobRequest;
import com.acme.camera.api.dto.TriangulationJobRequest;
import com.acme.camera.core.Extrinsics;
import com.acme.camera.core.Intrinsics;
import com.acme.camera.core.PinholeProjector;
import com.acme.camera.core.Point3D;
import com.acme.camera.core.ProjectionResult;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;

/**
 * Registry of everything the service ships pre-registered: named intrinsics presets and
 * the known cube calibration example. Read-only at runtime; the demo matched pairs are
 * computed once at startup by projecting the cube corners through the two demo cameras
 * with the real projection pipeline.
 */
@Component
public class PresetRegistry {

    private final List<NamedIntrinsics> intrinsicsPresets = List.of(
            new NamedIntrinsics("hd720-f800", DemoRig.INTRINSICS),
            new NamedIntrinsics("vga-f520", new Intrinsics(520.0, 520.0, 320.0, 240.0, 640, 480)));

    private final CubeDemo cubeDemo;

    public PresetRegistry(PinholeProjector projector) {
        List<Point3D> corners = DemoRig.cubeCorners();

        // Projection job: the cube corners expressed in camera-1 coordinates.
        List<Point3DDto> cam1Points = new ArrayList<>(corners.size());
        for (Point3D corner : corners) {
            Point3D p = DemoRig.CAMERA1.transform(corner);
            cam1Points.add(new Point3DDto(p.x(), p.y(), p.z()));
        }
        ProjectionJobRequest projectionJob = new ProjectionJobRequest(
                intrinsicsDto(), distortionDto(), cam1Points);

        // Triangulation job: both demo cameras plus the matched pixel pairs produced by
        // projecting each corner into each view.
        List<MatchDto> matches = new ArrayList<>(corners.size());
        for (Point3D corner : corners) {
            ProjectionResult inCam1 = projector.project(
                    DemoRig.CAMERA1.transform(corner), DemoRig.INTRINSICS, DemoRig.DISTORTION);
            ProjectionResult inCam2 = projector.project(
                    DemoRig.CAMERA2.transform(corner), DemoRig.INTRINSICS, DemoRig.DISTORTION);
            matches.add(new MatchDto(
                    new Point2DDto(inCam1.u(), inCam1.v()),
                    new Point2DDto(inCam2.u(), inCam2.v())));
        }
        TriangulationJobRequest triangulationJob = new TriangulationJobRequest(
                new CameraDto(intrinsicsDto(), distortionDto(), extrinsicsDto(DemoRig.CAMERA1)),
                new CameraDto(intrinsicsDto(), distortionDto(), extrinsicsDto(DemoRig.CAMERA2)),
                matches);

        this.cubeDemo = new CubeDemo(
                "Unit cube (side 1, centered at world origin) observed by two cameras. "
                        + "All 8 corners project inside the 1280x720 image in both views. "
                        + "POST cubeDemo.projectionJob to /api/jobs/projection or "
                        + "cubeDemo.triangulationJob to /api/jobs/triangulation to verify.",
                corners, projectionJob, triangulationJob);
    }

    public List<NamedIntrinsics> intrinsicsPresets() {
        return intrinsicsPresets;
    }

    public CubeDemo cubeDemo() {
        return cubeDemo;
    }

    private static IntrinsicsDto intrinsicsDto() {
        return new IntrinsicsDto(
                DemoRig.INTRINSICS.fx(), DemoRig.INTRINSICS.fy(),
                DemoRig.INTRINSICS.cx(), DemoRig.INTRINSICS.cy(),
                DemoRig.INTRINSICS.width(), DemoRig.INTRINSICS.height());
    }

    private static DistortionDto distortionDto() {
        return new DistortionDto(
                DemoRig.DISTORTION.k1(), DemoRig.DISTORTION.k2(),
                DemoRig.DISTORTION.p1(), DemoRig.DISTORTION.p2());
    }

    private static ExtrinsicsDto extrinsicsDto(Extrinsics e) {
        return new ExtrinsicsDto(e.rotation(), e.translation());
    }
}
