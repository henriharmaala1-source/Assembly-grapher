plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.kestrel.navviz"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.kestrel.navviz"
        minSdk = 26          // ARCore needs 24+; CameraX/TFLite comfortable at 26
        targetSdk = 34
        versionCode = 1
        versionName = "0.1-scaffold"
        ndk { abiFilters += listOf("arm64-v8a") }   // modern phones only; keeps the build lean
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    buildTypes {
        release { isMinifyEnabled = false }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
    // midas_small.onnx ships uncompressed so it can be mapped/read efficiently.
    androidResources { noCompress += "onnx" }
}

dependencies {
    val cameraxVersion = "1.3.4"
    implementation("androidx.camera:camera-core:$cameraxVersion")
    implementation("androidx.camera:camera-camera2:$cameraxVersion")
    implementation("androidx.camera:camera-lifecycle:$cameraxVersion")
    implementation("androidx.camera:camera-view:$cameraxVersion")

    // ONNX Runtime, not TFLite: runs the SAME midas_small.onnx the desktop
    // tools (tilt_bench.py / spin_map.py) already use, via OpenCV DNN's ONNX
    // path there and ORT's NNAPI execution provider here — no separate TFLite
    // model to source, no preprocessing to reconcile (see MidasDepth.kt).
    implementation("com.microsoft.onnxruntime:onnxruntime-android:1.26.0")

    // ARCore — used by the Stage-3 ArCorePoseProvider upgrade (see README).
    implementation("com.google.ar:core:1.44.0")

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
}
