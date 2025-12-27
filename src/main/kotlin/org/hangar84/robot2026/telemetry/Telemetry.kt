package org.hangar84.robot2026.telemetry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Telemetry {
    
    fun pose(prefix: String, pose: Pose2d) {
        SmartDashboard.putNumber("$prefix/X", pose.x)
        SmartDashboard.putNumber("$prefix/Y", pose.y)
        SmartDashboard.putNumber("$prefix/HeadingDeg", pose.rotation.degrees)
    }

    fun gyro(prefix: String, yaw: Rotation2d, yawRateDegPerSec: Double) {
        SmartDashboard.putNumber("$prefix/YawDeg", yaw.degrees)
        SmartDashboard.putNumber("$prefix/YawRateDegPerSec", yawRateDegPerSec)
    }
    
    fun chassisVel(prefix: String, vx: Double, vy: Double, omega: Double) {
        SmartDashboard.putNumber("$prefix/Vx", vx)
        SmartDashboard.putNumber("$prefix/Vy", vy)
        SmartDashboard.putNumber("$prefix/Omega", omega)
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

    fun num(key: String, value: Double) =
        SmartDashboard.putNumber(key, value)

    fun bool(key: String, value: Boolean) =
        SmartDashboard.putBoolean(key, value)
}