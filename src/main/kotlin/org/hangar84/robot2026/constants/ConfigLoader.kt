package org.hangar84.robot2026.constants

import edu.wpi.first.wpilibj.RobotBase
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.nio.file.Files
import java.nio.file.Path


data class Hinge(
    val hingeMotorId: Int
)

data class Pneumatics(
    val revPHId: Int,         // <-- your REVPH CanID
    val aExtendChannel: Int,    // solenoid channel
    val aRetractChannel: Int,
    val bExtendChannel: Int,      // solenoid channel
    val bRetractChannel: Int
)

data class Intake(
    val leftIntakeMotorID: Int,
    val rightIntakeMotorID: Int
)

data class Launcher(
    val leftLauncherId: Int,
    val rightLauncherId: Int
)

data class Mecanum(
    val frontLeftId: Int,
    val frontRightId: Int,
    val rearLeftId: Int,
    val rearRightId: Int
)

data class MaxConfig(
    val inverted: Boolean,
    val currentLimit: Int
)

data class SharedConfig(
    val hinge: Hinge,
    val intake: Intake,
    val pneumatics: Pneumatics,
    val launcher: Launcher,
    val max_config: MaxConfig
)

data class perRobotConfig(
    val swerve: Swerve? = null,
    val mecanum: Mecanum? = null
)

data class RootConfig(
    val shared: SharedConfig,
    val swerve: perRobotConfig,
    val mecanum: perRobotConfig
)

data class Swerve(
    val frontLeftDrivingId: Int,
    val frontLeftTurningId: Int,

    val frontRightDrivingId: Int,
    val frontRightTurningId: Int,

    val rearLeftDrivingId: Int,
    val rearLeftTurningId: Int,

    val rearRightDrivingId: Int,
    val rearRightTurningId: Int
)

object ConfigLoader {
    private val path: Path = if (!RobotBase.isSimulation())
        Path.of("/home/lvuser/deploy/config/robot_config.json")
    else {
        Path.of("src/main/deploy/config/robot_config.json")
    }
    private val parser = JSONParser()

    private fun JSONObject.int(key: String): Int {
        val v = this[key] ?: error("Missing key '$key' in object keys=${this.keys}")
        return when (v) {
            is Long -> v.toInt()
            is Int -> v
            is Double -> v.toInt()
            is Number -> v.toInt()
            else -> error("Key '$key' is not a number: value=$v type=${v::class}")
        }
    }

    private fun JSONObject.bool(key: String): Boolean {
        val v = this[key] ?: error("Missing key '$key' in object keys=${this.keys}")
        return when (v) {
            is Boolean -> v
            is String -> v.toBooleanStrictOrNull()
                ?: error("Key '$key' is not a valid boolean string: $v")
            else -> error("Key '$key' is not a boolean: value=$v type=${v::class}")
        }
    }

    // cache the parsed root so we don't reread the file constantly
    private val root: JSONObject by lazy {
        val text = Files.readString(path)
        parser.parse(text) as JSONObject
    }

    fun sectionFor(robotType: RobotType): JSONObject {
        val key = when (robotType) {
            RobotType.SWERVE -> "swerve"
            RobotType.MECANUM -> "mecanum"
        }
        return root[key] as JSONObject
    }

    fun sharedJson(): JSONObject = root["shared"] as JSONObject

    fun loadShared(): SharedConfig {
        val s = sharedJson()

        val hingeJ = s["hinge"] as JSONObject
        val intakeJ = s["intake"] as JSONObject
        val launcherJ = s["launcher"] as JSONObject
        val pneuJ = s["pneumatics"] as JSONObject
        val maxcfgJ = s["max_config"] as JSONObject

        return SharedConfig(
            hinge = Hinge(hingeMotorId = hingeJ.int("hingeMotorId")),
            intake = Intake(
                leftIntakeMotorID = intakeJ.int("leftIntakeMotorId"),
                rightIntakeMotorID = intakeJ.int("rightIntakeMotorId")
            ),
            launcher = Launcher(
            leftLauncherId = launcherJ.int("leftLauncherMotorId"),
            rightLauncherId = launcherJ.int("rightLauncherMotorId")
            ),
            pneumatics = Pneumatics(
                revPHId = pneuJ.int("revPHId"),
                aExtendChannel = pneuJ.int("aExtendChannel"),
                aRetractChannel = pneuJ.int("aRetractChannel"),
                bExtendChannel = pneuJ.int("bExtendChannel"),
                bRetractChannel = pneuJ.int("bRetractChannel")
            ),
            max_config = MaxConfig(
                maxcfgJ.bool("inverted"),
                maxcfgJ.int("currentLimit")
            )
        )
    }

    fun loadPerRobot(robotType: RobotType): perRobotConfig {
        val r = sectionFor(robotType)

        val swerve: Swerve? = (r["swerve"] as JSONObject?)?.let { j ->
            Swerve(
                frontLeftDrivingId = j.int("frontLeftDrivingId"),
                frontLeftTurningId = j.int("frontLeftTurningId"),
                frontRightDrivingId = j.int("frontRightDrivingId"),
                frontRightTurningId = j.int("frontRightTurningId"),
                rearLeftDrivingId = j.int("rearLeftDrivingId"),
                rearLeftTurningId = j.int("rearLeftTurningId"),
                rearRightDrivingId = j.int("rearRightDrivingId"),
                rearRightTurningId = j.int("rearRightTurningId")
            )
        }

        val mecanum: Mecanum? = (r["mecanum"] as JSONObject?)?.let { j ->
            Mecanum(
                frontLeftId = j.int("frontLeftId"),
                frontRightId = j.int("frontRightId"),
                rearLeftId = j.int("rearLeftId"),
                rearRightId = j.int("rearRightId")
            )
        }

        return perRobotConfig(swerve = swerve, mecanum = mecanum)
    }
}