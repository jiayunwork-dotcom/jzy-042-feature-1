package com.acme.camera.config;

import com.acme.camera.calibration.CameraCalibrator;
import com.acme.camera.core.DltTriangulator;
import com.acme.camera.core.PinholeProjector;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

/** Wires the stateless geometry kernels as singleton beans shared by all services. */
@Configuration
public class CoreConfig {

    @Bean
    public PinholeProjector pinholeProjector() {
        return new PinholeProjector();
    }

    @Bean
    public DltTriangulator dltTriangulator() {
        return new DltTriangulator();
    }

    @Bean
    public CameraCalibrator cameraCalibrator(PinholeProjector projector) {
        return new CameraCalibrator(projector);
    }
}
