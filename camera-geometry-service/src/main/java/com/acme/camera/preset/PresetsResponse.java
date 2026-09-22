package com.acme.camera.preset;

import java.util.List;

/** Read-only view of everything the service ships pre-registered. */
public record PresetsResponse(List<NamedIntrinsics> intrinsicsPresets, CubeDemo cubeDemo) {
}
