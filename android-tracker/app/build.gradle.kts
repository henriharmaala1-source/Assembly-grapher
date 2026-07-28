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
    // UVC over USB Host — this phone does NOT expose the dongle via Camera2
    // external (many don't), so we go through libuvc like the generic USB-camera
    // app does. herohan/UVCAndroid is a single AAR on MAVEN CENTRAL (unlike the
    // AUSBC JitPack multi-module build, which fails to resolve its libuvc module
    // — jiangdongguo/AndroidUSBCamera issues #727/#728). See UvcFrameSource.kt.
    // Camera2FrameSource.kt is kept as a built-in-camera fallback for phones
    // that DO expose UVC via Camera2.
    implementation("com.herohan:UVCAndroid:1.0.13")

    // Learned tracker (OnnxSiameseTracker.kt). ONNX Runtime rather than the
    // OpenCV Android SDK: ~15 MB against OpenCV's 30-40 MB per ABI, it exposes
    // NNAPI/XNNPACK, and OpenCV's own model card warns its DNN backend has
    // "poor support for the transformer architecture for now".
    implementation("com.microsoft.onnxruntime:onnxruntime-android:1.28.0")

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
    implementation("androidx.appcompat:appcompat:1.7.0")
}
