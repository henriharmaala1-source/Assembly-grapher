// Top-level build file. Plugin versions are pinned here; the app module applies
// them.
//
// STABLE AGP 8.x line — deliberately NOT AGP 9. AGP 9.x (Jan 2026) removed the
// old BaseExtension DSL that the classic org.jetbrains.kotlin.android plugin
// casts to, so AGP 9 + that plugin throws "ApplicationExtensionImpl cannot be
// cast to BaseExtension". Rather than take on AGP 9's built-in-Kotlin migration
// mid-project, we pin the mature, universally-used pairing:
//   AGP 8.7.2  +  Gradle 8.9 (gradle/wrapper/gradle-wrapper.properties)  +  Kotlin 2.0.20
// The gradle wrapper is the key piece: it FORCES Gradle 8.9, so AGP 8.x never
// meets the Project.exec() removal in Gradle 9 that started the version chase.
plugins {
    id("com.android.application") version "8.7.2" apply false
    id("org.jetbrains.kotlin.android") version "2.0.20" apply false
}
