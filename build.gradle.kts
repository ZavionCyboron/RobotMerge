import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    java
    kotlin("jvm") version "2.1.10"
    id("edu.wpi.first.GradleRIO") version "2026.2.1"
    idea
}

val javaVersion: JavaVersion by extra { JavaVersion.VERSION_17 }
val javaLanguageVersion: JavaLanguageVersion by extra { JavaLanguageVersion.of(javaVersion.toString()) }
val jvmVendor: JvmVendorSpec by extra { JvmVendorSpec.ADOPTIUM }
val kotlinJvmTarget: JvmTarget = JvmTarget.fromTarget(javaVersion.toString())

deploy {
    targets {
        val roborio by register<RoboRIO>("roborio") {
            team = frc.teamNumber
            debug = frc.getDebugOrDefault(false)
        }

        roborio.artifacts {
            register<FRCJavaArtifact>("frcJava") {
                setJarTask(tasks.jar)
            }

            register<FileTreeArtifact>("frcStaticFileDeploy") {
                files.set(project.fileTree("src/main/deploy") { include("**/*") })
                directory.set("/home/lvuser/deploy")
                deleteOldFiles.set(true)
            }
        }
    }
}

wpi {
    with(java) {
        debugJni = false
        configureExecutableTasks(tasks.jar.get())
        configureTestTasks(tasks.test.get())
    }

    with(sim) {
        addGui().apply {
            defaultEnabled = true
        }
        addDriverstation()
    }
}

val includeDesktopSupport = true

dependencies {
    annotationProcessor(wpi.java.deps.wpilibAnnotations())
    implementation(wpi.java.deps.wpilib())
    implementation(wpi.java.vendor.java())

    roborioDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio))
    roborioDebug(wpi.java.vendor.jniDebug(NativePlatforms.roborio))

    roborioRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio))
    roborioRelease(wpi.java.vendor.jniRelease(NativePlatforms.roborio))

    nativeDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop))
    nativeDebug(wpi.java.vendor.jniDebug(NativePlatforms.desktop))
    simulationDebug(wpi.sim.enableDebug())

    nativeRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop))
    nativeRelease(wpi.java.vendor.jniRelease(NativePlatforms.desktop))
    simulationRelease(wpi.sim.enableRelease())

    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter-api")
    testImplementation("org.junit.jupiter:junit-jupiter-params")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine")
}

java {
    toolchain {
        languageVersion = javaLanguageVersion
        vendor = jvmVendor
    }
}

kotlin {
    compilerOptions {
        jvmTarget = kotlinJvmTarget
    }

    jvmToolchain {
        languageVersion = javaLanguageVersion
        vendor = jvmVendor
    }
}
tasks {

    test {
        useJUnitPlatform()
        systemProperty("junit.jupiter.extensions.autodetection.enabled", "true")
    }
    compileJava {
        options.encoding = Charsets.UTF_8.name()
        options.compilerArgs.add("-XDstringConcat=inline")
    }

    jar {
        group = "build"
        manifest(GradleRIOPlugin.javaManifest(providers.gradleProperty("ROBOT_MAIN_CLASS").orNull))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE

        from({ configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) } })

        from({ sourceSets.main.get().allSource })
    }
}

idea {
    project {
        languageLevel = IdeaLanguageLevel(javaVersion)
    }

    module {
        isDownloadJavadoc = true
        isDownloadSources = true

        excludeDirs.add(file(".run"))
        excludeDirs.add(file(".vscode"))
    }
}

fun DependencyHandler.addDependencies(
    configurationName: String,
    dependencies: List<Provider<String>>,
) = dependencies.forEach { add(configurationName, it) }

fun DependencyHandler.roborioDebug(dependencies: List<Provider<String>>) = addDependencies("roborioDebug", dependencies)

fun DependencyHandler.roborioRelease(dependencies: List<Provider<String>>) = addDependencies("roborioRelease", dependencies)

fun DependencyHandler.nativeDebug(dependencies: List<Provider<String>>) = addDependencies("nativeDebug", dependencies)

fun DependencyHandler.simulationDebug(dependencies: List<Provider<String>>) = addDependencies("simulationDebug", dependencies)

fun DependencyHandler.nativeRelease(dependencies: List<Provider<String>>) = addDependencies("nativeRelease", dependencies)

fun DependencyHandler.simulationRelease(dependencies: List<Provider<String>>) = addDependencies("simulationRelease", dependencies)

fun DependencyHandler.implementation(dependencies: List<Provider<String>>) = dependencies.forEach { implementation(it) }

fun DependencyHandler.annotationProcessor(dependencies: List<Provider<String>>) = dependencies.forEach { annotationProcessor(it) }
