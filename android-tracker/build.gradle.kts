// Top-level build file. Plugin versions pinned here; the app module applies them.
//
// This is the SAME hard-won pairing navviz landed on after the AGP 9 disaster —
// carried over verbatim so this app never repeats that fight:
//   AGP 8.7.2  +  Gradle 8.9 (gradle/wrapper/gradle-wrapper.properties)  +  Kotlin 2.0.20
// AGP 9.x removed the BaseExtension DSL the classic kotlin.android plugin casts
// to; the Gradle-8.9 wrapper forces the 8.x line so we never meet it. If Android
// Studio offers to upgrade AGP/Gradle: DECLINE.
plugins {
    id("com.android.application") version "8.7.2" apply false
    id("org.jetbrains.kotlin.android") version "2.0.20" apply false
}
