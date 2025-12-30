package org.hangar84.robot2026.io.real

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.GyroIO.Inputs

class AdisGyroIO: GyroIO {
    private val imu = ADIS16470_IMU()

    override fun updateInputs(inputs: Inputs) {
        inputs.yaw = Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ))

        inputs.yawRateDegPerSec
    }

    override fun zeroYaw() {
        imu.reset()
    }
}