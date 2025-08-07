plugins {
    id("java-library")
    id("maven-publish")
}
dependencies {
    compileOnly(libs.annotations)
}

java {
    toolchain.languageVersion.set(JavaLanguageVersion.of(17))
    withSourcesJar()
    withJavadocJar()
}

publishing {
    publications {
        register<MavenPublication>("release") {
            groupId = "com.pedropathing"
            artifactId = "core"
            version = property("version") as String

            from(components["java"])
        }
    }

    repositories {
        maven {
            name = "publishing"
            url = uri("../../maven.pedropathing.com")
        }
    }
}