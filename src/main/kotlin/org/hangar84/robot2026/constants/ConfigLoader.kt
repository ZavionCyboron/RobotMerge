package org.hangar84.robot2026.constants

import edu.wpi.first.wpilibj.RobotBase
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser
import java.nio.file.Files
import java.nio.file.Path


data class Hinge(
    val hingeMotorId: Int,
    val maxLimitSwitchOneDio: Int? = null,
    val maxLimitSwitchTwoDio: Int? = null
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

/*data class RootConfig(
    val shared: SharedConfig,
    val swerve: perRobotConfig,
    val mecanum: perRobotConfig
)*/

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

    private fun JSONObject.optInt(key: String): Int? {
        val v = this[key] ?: return null
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

        val driveRoot = root["robotType"]as? JSONObject
            ?: error("Missing 'robotType' object at root. keys=${root.keys}")

        return driveRoot[key] as? JSONObject
            ?: error("Missing drive section '$key' inside 'robotType'. keys=${driveRoot.keys}")
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
            hinge = Hinge(
                hingeMotorId = hingeJ.int("hingeMotorId"),
                maxLimitSwitchOneDio = hingeJ.optInt("maxLimitSwitchOneDio"),
                maxLimitSwitchTwoDio = hingeJ.optInt("maxLimitSwitchTwoDio")
            ),
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

        return when (robotType) {
            RobotType.SWERVE -> perRobotConfig(
                swerve = Swerve(
                    frontLeftDrivingId = r.int("frontLeftDrivingId"),
                    frontLeftTurningId = r.int("frontLeftTurningId"),
                    frontRightDrivingId = r.int("frontRightDrivingId"),
                    frontRightTurningId = r.int("frontRightTurningId"),
                    rearLeftDrivingId = r.int("rearLeftDrivingId"),
                    rearLeftTurningId = r.int("rearLeftTurningId"),
                    rearRightDrivingId = r.int("rearRightDrivingId"),
                    rearRightTurningId = r.int("rearRightTurningId"),
                )
            )

            RobotType.MECANUM -> perRobotConfig(
                mecanum = Mecanum(
                    frontLeftId = r.int("frontLeftId"),
                    frontRightId = r.int("frontRightId"),
                    rearLeftId = r.int("rearLeftId"),
                    rearRightId = r.int("rearRightId"),
                )
            )
        }
    }
}