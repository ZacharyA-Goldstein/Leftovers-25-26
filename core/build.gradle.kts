plugins {
    id("java-library")
    id("maven-publish")
}
dependencies {
    compileOnly(libs.annotations)
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