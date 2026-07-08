// Top-level build file. Plugin versions are pinned here; the app module applies
// them.
//
// AGP 9.x requires Gradle 9.1.0+ (see gradle/wrapper/gradle-wrapper.properties)
// — AGP 8.5.2 predates Gradle 9 support and its native-build (CMake) code calls
// a Project.exec() overload Gradle 9.0 removed, which is the exact
// NoSuchMethodError this version bump fixes. AGP 9.0+ also bundles its own
// Kotlin support and wants KGP >= 2.2.10; android.builtInKotlin=false in
// gradle.properties keeps the explicit org.jetbrains.kotlin.android plugin
// below working without ALSO migrating to that new config style in this same
// fix (opt-out still supported through AGP 9.x, removed in AGP 10).
plugins {
    id("com.android.application") version "9.2.0" apply false
    id("org.jetbrains.kotlin.android") version "2.2.10" apply false
}
