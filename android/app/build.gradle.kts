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
    // MiDaS .tflite ships uncompressed so it can be memory-mapped at runtime.
    androidResources { noCompress += "tflite" }
}

dependencies {
    val cameraxVersion = "1.3.4"
    implementation("androidx.camera:camera-core:$cameraxVersion")
    implementation("androidx.camera:camera-camera2:$cameraxVersion")
    implementation("androidx.camera:camera-lifecycle:$cameraxVersion")
    implementation("androidx.camera:camera-view:$cameraxVersion")

    implementation("org.tensorflow:tensorflow-lite:2.14.0")
    implementation("org.tensorflow:tensorflow-lite-gpu:2.14.0")

    // ARCore — used by the Stage-3 ArCorePoseProvider upgrade (see README).
    implementation("com.google.ar:core:1.44.0")

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
}
