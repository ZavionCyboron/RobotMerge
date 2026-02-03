package org.hangar84.robot2026.telemetry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
object SimTelemetry {
    fun pose(prefix: String, pose: Pose2d) {
        SmartDashboard.putNumber("$prefix/X", pose.x)
        SmartDashboard.putNumber("$prefix/Y", pose.y)
        SmartDashboard.putNumber("$prefix/RotDeg", pose.rotation.degrees)
    }

    fun poseError(prefix: String, truth: Pose2d, est: Pose2d) {
        SmartDashboard.putNumber("$prefix/dX", est.x - truth.x)
        SmartDashboard.putNumber("$prefix/dY", est.y - truth.y)
        SmartDashboard.putNumber("$prefix/dYawDeg", est.rotation.minus(truth.rotation).degrees)
    }

    fun poseCompare(prefix: String, truth: Pose2d, est: Pose2d) {
        pose("$prefix/Truth", truth)
        pose("$prefix/Estimated", est)
        num("$prefix/ErrorXY", truth.translation.getDistance(est.translation))
        num("$prefix/ErrorDeg", truth.rotation.minus(est.rotation).degrees)
    }

    fun chassisVel(prefix: String, vx: Double, vy: Double, omega: Double) {
        SmartDashboard.putNumber("$prefix/Vx", vx)
        SmartDashboard.putNumber("$prefix/Vy", vy)
        SmartDashboard.putNumber("$prefix/Omega", omega)
    }

    fun speedMPS(prefix: String, fl: Double, fr: Double, rl: Double, rr: Double) {
        SmartDashboard.putNumber("$prefix/FL", fl)
        SmartDashboard.putNumber("$prefix/FR", fr)
        SmartDashboard.putNumber("$prefix/RL", rl)
        SmartDashboard.putNumber("$prefix/RR", rr)
    }

    // Swerve only
    fun angleDeg(prefix: String, fl: Double, fr: Double, rl: Double, rr: Double) {
        SmartDashboard.putNumber("$prefix/FL", fl)
        SmartDashboard.putNumber("$prefix/FR", fr)
        SmartDashboard.putNumber("$prefix/RL", rl)
        SmartDashboard.putNumber("$prefix/RR", rr)
    }

    fun wheelEncoders(prefix: String, flPos: Double, frPos: Double, rlPos: Double, rrPos: Double,
                      flVel: Double, frVel: Double, rlVel: Double, rrVel: Double) {
        SmartDashboard.putNumber("$prefix/Pos/FL", flPos)
        SmartDashboard.putNumber("$prefix/Pos/FR", frPos)
        SmartDashboard.putNumber("$prefix/Pos/RL", rlPos)
        SmartDashboard.putNumber("$prefix/Pos/RR", rrPos)

        SmartDashboard.putNumber("$prefix/Vel/FL", flVel)
        SmartDashboard.putNumber("$prefix/Vel/FR", frVel)
        SmartDashboard.putNumber("$prefix/Vel/RL", rlVel)
        SmartDashboard.putNumber("$prefix/Vel/RR", rrVel)
    }

    fun gyro(prefix: String, yaw: Rotation2d, yawRateDegPerSec: Double) {
        SmartDashboard.putNumber("$prefix/YawDeg", yaw.degrees)
        SmartDashboard.putNumber("$prefix/YawRateDegPerSec", yawRateDegPerSec)
    }
    fun gyroTrue(prefix: String, trueYaw: Rotation2d, yaw: Rotation2d, trueYawRateDegPerSec: Double) {
        SmartDashboard.putNumber("$prefix/TrueYawDeg", trueYaw.degrees)
        SmartDashboard.putNumber("$prefix/MeasuredYawDeg", yaw.degrees)
        SmartDashboard.putNumber("$prefix/TrueYawRateDegPerSec", trueYawRateDegPerSec)
    }
    fun num(key: String, value: Double) = SmartDashboard.putNumber(key, value)
    fun bool(key: String, value: Boolean) = SmartDashboard.putBoolean(key, value)
}