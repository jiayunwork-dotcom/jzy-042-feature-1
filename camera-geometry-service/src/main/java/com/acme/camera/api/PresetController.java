package com.acme.camera.api;

import com.acme.camera.preset.PresetRegistry;
import com.acme.camera.preset.PresetsResponse;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** Read-only echo of the registered intrinsics presets and the known cube calibration example. */
@RestController
@RequestMapping("/api")
public class PresetController {

    private final PresetRegistry presetRegistry;

    public PresetController(PresetRegistry presetRegistry) {
        this.presetRegistry = presetRegistry;
    }

    @GetMapping("/presets")
    public PresetsResponse presets() {
        return new PresetsResponse(presetRegistry.intrinsicsPresets(), presetRegistry.cubeDemo());
    }
}
