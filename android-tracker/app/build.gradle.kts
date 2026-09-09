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

    // arm64 only. The bytedeco OpenCV artifact carries 82 native libraries per
    // ABI; without this the APK would balloon by hundreds of megabytes for
    // architectures this rig never runs on.
    defaultConfig { ndk { abiFilters += "arm64-v8a" } }

    packaging {
        jniLibs {
            // bytedeco and onnxruntime both ship libc++_shared.so.
            pickFirsts += "**/libc++_shared.so"
            // Everything CSRT does not need. opencv_tracking pulls core,
            // imgproc, video, features2d, calib3d, dnn and flann transitively;
            // the rest is dead weight in the APK.
            excludes += listOf("**/libopencv_gapi.so", "**/libopencv_stitching.so",
                               "**/libopencv_photo.so", "**/libopencv_ml.so",
                               "**/libopencv_objdetect.so", "**/libopencv_highgui.so",
                               "**/libjniopencv_gapi.so", "**/libjniopencv_stitching.so",
                               "**/libjniopencv_photo.so", "**/libjniopencv_ml.so",
                               "**/libjniopencv_objdetect.so", "**/libjniopencv_highgui.so")
        }
        resources { excludes += "META-INF/*.kotlin_module" }
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

    // CSRT (CsrtTracker.kt) — the A/B that scored 88% on the battery against
    // this project's 70%. NOT available from the official OpenCV Android AAR:
    // that ships only MIL/GOTURN/DaSiamRPN/Nano/Vit, because CSRT lives in
    // opencv_contrib which the official Android build excludes (verified by
    // unpacking org.opencv:opencv:4.12.0). bytedeco does carry it —
    // libopencv_tracking.so, 1.9 MB, arm64-v8a.
    //
    // arm64 ONLY, deliberately: the platform artifact would pull every ABI and
    // add hundreds of MB. UNVERIFIED — this was never compiled; if it fails to
    // resolve, delete these four lines and CsrtTracker.kt.
    implementation("org.bytedeco:javacpp:1.5.13")
    implementation("org.bytedeco:javacpp:1.5.13:android-arm64")
    implementation("org.bytedeco:opencv:4.13.0-1.5.13")
    implementation("org.bytedeco:opencv:4.13.0-1.5.13:android-arm64")

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
    implementation("androidx.appcompat:appcompat:1.7.0")
}
