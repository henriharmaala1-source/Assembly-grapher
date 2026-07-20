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
    // UVC / USB-camera: ingests the analog capture dongle (same one that worked
    // in the generic USB-camera app). AUSBC (AndroidUSBCamera) is actively
    // maintained and on JitPack; it opens the device and delivers raw frame
    // callbacks — which is ALL we use it for (we render + track ourselves).
    // NOTE: this is the one seam that can't be compile-checked here — verify the
    // exact frame-callback API against the library version on device (see
    // camera/UvcFrameSource.kt and README).
    implementation("com.github.jiangdongguo.AndroidUSBCamera:libausbc:3.3.3")

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-ktx:1.9.2")
    implementation("androidx.appcompat:appcompat:1.7.0")
}
