plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.kestrel.tracker"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.kestrel.tracker"
        minSdk = 26
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"
    }

    buildTypes {
        release { isMinifyEnabled = false }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
}

dependencies {
    // No external UVC library. The capture dongle is ingested through the
    // platform Camera2 API as an EXTERNAL camera (the same reason a generic
    // USB-camera app can see it) — zero extra dependencies, nothing to resolve
    // from JitPack, and it falls back to the built-in camera when no dongle is
    // attached so the app always runs. See camera/Camera2FrameSource.kt.
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
    implementation("androidx.appcompat:appcompat:1.7.0")
}
