package org.hangar84.robot2026.io.real

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.GyroIO.Inputs

class AdisGyroIO: GyroIO {
    private val imu = ADIS16470_IMU()
    private var yawOffsetDeg = 0.0

    override fun updateInputs(inputs: GyroIO.Inputs) {
        val rawYawDeg = imu.getAngle(ADIS16470_IMU.IMUAxis.kZ)
        inputs.yaw = Rotation2d.fromDegrees(rawYawDeg - yawOffsetDeg)
        inputs.yawRateDegPerSec = imu.getRate(ADIS16470_IMU.IMUAxis.kZ)
    }

    override fun zeroYaw() {
        // Make "current robot forward" become 0°
        val rawYawDeg = imu.getAngle(ADIS16470_IMU.IMUAxis.kZ)
        yawOffsetDeg = rawYawDeg
    }

    fun setYawAdjustmentDegrees(adjustDeg: Double) {
        yawOffsetDeg += adjustDeg
    }
}