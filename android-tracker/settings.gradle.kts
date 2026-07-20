pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        // JitPack — hosts the UVC (USB-camera) library used to ingest the
        // analog capture dongle. See app/build.gradle.kts.
        maven { url = uri("https://jitpack.io") }
    }
}
rootProject.name = "kestrel-tracker"
include(":app")
