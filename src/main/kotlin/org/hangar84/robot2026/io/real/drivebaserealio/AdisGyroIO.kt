package org.hangar84.robot2026.io.real.drivebaserealio

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import org.hangar84.robot2026.io.interfaces.drivebaseio.GyroIO

class AdisGyroIO: GyroIO {
    private val imu = ADIS16470_IMU()
    private var yawOffsetDeg = 0.0

    override fun updateInputs(inputs: GyroIO.Inputs) {
        val rawRate = imu.getRate(IMUAxis.kZ)
        val filteredRate = when {
            kotlin.math.abs(rawRate) < 1.0 -> 0.0
            rawRate > 10.0 -> 10.0
            rawRate < -10.0 -> -10.0
            else -> rawRate
        }

        val rawYawDeg = imu.getAngle(IMUAxis.kZ) - yawOffsetDeg
        val wrappedYawDeg = edu.wpi.first.math.MathUtil.inputModulus(rawYawDeg, -180.0, 180.0)

        inputs.yaw = Rotation2d.fromDegrees(wrappedYawDeg)
        inputs.yawRateDegPerSec = filteredRate
    }
    override fun zeroYaw() {
        // Make "current robot forward" become 0°
        val rawYawDeg = imu.getAngle(IMUAxis.kZ)
        yawOffsetDeg = rawYawDeg
    }

    fun setYawAdjustmentDegrees(adjustDeg: Double) {
        yawOffsetDeg += adjustDeg
    }
}