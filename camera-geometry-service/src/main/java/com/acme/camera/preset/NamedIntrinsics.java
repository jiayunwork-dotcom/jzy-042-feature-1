package com.acme.camera.preset;

import com.acme.camera.core.Intrinsics;

/** A named, ready-to-use intrinsics preset. */
public record NamedIntrinsics(String name, Intrinsics intrinsics) {
}
