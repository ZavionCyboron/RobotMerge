package org.hangar84.robot2026.telemetry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import org.hangar84.robot2026.RobotContainer.robotType

object TelemetryRouter {
    private val isSim = RobotBase.isSimulation()
    private val lastPublishTime = mutableMapOf<String, Double>()

    private var base = robotType.name

    private fun shouldPublish(group: String): Boolean {
        if (!TelemetryConfig.enabled(group, true)) return false

        val hz = TelemetryConfig.rateHz(group) ?: return true
        if (hz <= 0) return false

        val now = Timer.getFPGATimestamp()
        val minPeriod = 1.0 / hz
        val last = lastPublishTime[group] ?: Double.NEGATIVE_INFINITY

        if ((now - last) >= minPeriod) {
            lastPublishTime[group] = now
            return true
        }
        return false
    }

    fun setBase(name: String) {
        base = name
    }

    private fun publish(key: String, value: Double) {
        TelemetrySinks.publishNumber(key, value)
    }

    fun pose(pose: Pose2d) {
        if (!shouldPublish("pose")) return
        val prefix = TelemetryConfig.prefix("pose", "Pose")

        publish("$base/Pose/X", pose.x)
        publish("$base/Pose/Y", pose.y)
        publish("$base/Pose/Rot", pose.rotation.degrees)

        if (isSim) {
            SimTelemetry.pose(prefix, pose)
        } else {
            Telemetry.pose(prefix, pose)
        }
    }

    fun poseError(truth: Pose2d, est: Pose2d) {
        if (isSim) {
            if (!shouldPublish("pose")) return

            publish("$base/Pose/dX", truth.x)
            publish("$base/Pose/dY", est.y)
            publish("$base/Pose/dRot", est.rotation.minus(truth.rotation).degrees)

            SimTelemetry.poseError(
                TelemetryConfig.prefix("pose", "Pose"),
                truth, est
            )
        } else return
    }

    fun poseCompare(truth: Pose2d, est: Pose2d) {
        if (isSim) {
            if (!shouldPublish("pose")) return

            publish("$base/Pose/Truth", truth.x)
            publish("$base/Pose/Estimated", est.y)
            publish("$base/Pose/ErrorXY", truth.translation.getDistance(est.translation))
            publish("$base/Pose/ErrorDeg", truth.rotation.minus(est.rotation).degrees)

            SimTelemetry.poseCompare(
                TelemetryConfig.prefix("pose", "Pose"),
                truth, est
            )
        } else return
    }

    fun gyro(yaw: Rotation2d, yawRateDegPerSec: Double) {
        if (!shouldPublish("gyro")) return
        val prefix = TelemetryConfig.prefix("gyro", "Gyro")

        publish("$base/Gyro/YawDeg", yaw.degrees)
        publish("$base/Gyro/YawRateDegPerSec", yawRateDegPerSec)

        if (isSim) {
            SimTelemetry.gyro(prefix, yaw, yawRateDegPerSec)
        } else {
            Telemetry.gyro(prefix, yaw, yawRateDegPerSec)
        }
    }

    fun gyroTrue(trueYaw: Rotation2d, yaw: Rotation2d, trueYawRateDegPerSec: Double) {
        if (isSim) {
            if (!shouldPublish("gyro")) return

            publish("$base/TrueGyro/TrueYawDeg", trueYaw.degrees)
            publish("$base/TrueGyro/MeasuredYawDeg", yaw.degrees)
            publish("$base/TrueGyro/TrueYawRateDegPerSec", trueYawRateDegPerSec)

            SimTelemetry.gyroTrue(
                TelemetryConfig.prefix("gyro", "Gyro"),
                trueYaw, yaw, trueYawRateDegPerSec
            )
        } else return
    }

    fun chassisVel(vx: Double, vy: Double, omega: Double) {
        if (!shouldPublish("chassis")) return
        val prefix = TelemetryConfig.prefix("chassis", "Chassis")

        publish("$base/ChassisVel/VX", vx)
        publish("$base/ChassisVel/VY", vy)
        publish("$base/ChassisVel/Omega", omega)

        if (isSim) {
            SimTelemetry.chassisVel(prefix, vx, vy, omega)
        } else {
            Telemetry.chassisVel(prefix, vx, vy, omega)
        }
    }

    fun wheelEncoders(flPos: Double, frPos: Double, rlPos: Double, rrPos: Double,
               flVel: Double, frVel: Double, rlVel: Double, rrVel: Double
    ) {
        if (!shouldPublish("wheels")) return
        val prefix = TelemetryConfig.prefix("wheels", "Wheels")

        publish("$base/Pos/FL", flPos)
        publish("$base/Pos/FR", frPos)
        publish("$base/Pos/RL", rlPos)
        publish("$base/Pos/RR", rrPos)

        publish("$base/Vel/FL", flVel)
        publish("$base/Vel/FR", frVel)
        publish("$base/Vel/RL", rlVel)
        publish("$base/Vel/RR", rrVel)

        if (isSim) {
            SimTelemetry.wheelEncoders(
                prefix,
                flPos, frPos, rlPos, rrPos,
                flVel, frVel, rlVel, rrVel
            )
        } else {
            Telemetry.wheelEncoders(
                prefix,
                flPos, frPos, rlPos, rrPos,
                flVel, frVel, rlVel, rrVel
            )
        }
    }

    fun angleDeg(fl: Double, fr: Double, rl: Double, rr: Double) {
        if (isSim) {
            if (!shouldPublish("motors")) return

            publish("$base/AngleDeg/FL", fl)
            publish("$base/AngleDeg/FR", fr)
            publish("$base/AngleDeg/RL", rl)
            publish("$base/AngleDeg/RR", rr)


            SimTelemetry.angleDeg(
                TelemetryConfig.prefix("motors", "Motors"),
                fl, fr, rl, rr
            )
        } else return
    }

    fun launcher(
        leftAppliedOutput: Double, rightAppliedOutput: Double,
        leftVelocityRpm: Double, rightVelocityRpm: Double,
        leftCurrentAmps: Double, rightCurrentAmps: Double) {
            if (!shouldPublish("launcher")) return

            publish("$base/LeftOutput", leftAppliedOutput)
            publish("$base/RightOutput", rightAppliedOutput)
            publish("$base/LeftRPM", leftVelocityRpm)
            publish("$base/RightRPM", rightVelocityRpm)
            publish("$base/LeftCurrent", leftCurrentAmps)
            publish("$base/RightCurrent", rightCurrentAmps)

        if (isSim) {
            SimTelemetry.launcher(
                TelemetryConfig.prefix("launcher", "Launchers"),
                leftAppliedOutput, rightAppliedOutput,
                leftVelocityRpm, rightVelocityRpm,
                leftCurrentAmps, rightCurrentAmps
            )
        }
    }

    fun pneumatics(
        compressorEnabled: Boolean,
        extendSolenoidOn: Boolean,
        retractSolenoidOn: Boolean,
        state: String,
        pressurePsi: Double? = null
    ) {

        if (!shouldPublish("pneumatics")) return
        val prefix = TelemetryConfig.prefix("pneumatics", "Pneumatics")

        TelemetrySinks.publishBoolean("$base/CompressorEnabled", compressorEnabled)
        TelemetrySinks.publishBoolean("$base/ExtendSolenoidOn", extendSolenoidOn)
        TelemetrySinks.publishBoolean("$base/RetractSolenoidOn", retractSolenoidOn)
        TelemetrySinks.publishString("$base/State", state)

        if (isSim) {

            SimTelemetry.pneumatics(
                prefix,
                compressorEnabled, state,
                extendSolenoidOn, retractSolenoidOn, pressurePsi
            )
        } else {

            // optional if you have a pressure sensor later
            if (pressurePsi != null) {
                TelemetrySinks.publishNumber("$base/Pneumatics/PressurePsi", pressurePsi)
            }
        }
    }

    fun num(key: String, value: Double) {
        if (!shouldPublish("debug")) return

        if (isSim) {
            SimTelemetry.num(key, value)
        } else {
            Telemetry.num(key, value)
        }
    }

    fun bool(key: String, value: Boolean) {
        if (!shouldPublish("debug")) return

        if (isSim) {
            SimTelemetry.bool(key, value)
        } else {
            Telemetry.bool(key, value)
        }

    }
}