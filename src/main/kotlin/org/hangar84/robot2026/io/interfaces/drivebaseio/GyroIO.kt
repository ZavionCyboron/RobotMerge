package org.hangar84.robot2026.io.interfaces.drivebaseio

import edu.wpi.first.math.geometry.Rotation2d

interface GyroIO {
    data class Inputs(
        var yaw: Rotation2d = Rotation2d(),
        var yawRateDegPerSec: Double = 0.0
    )

    fun updateInputs(inputs: Inputs)
    fun zeroYaw()

    fun simulationPeriodic(dtSeconds: Double) {}

    fun setSimOmegaRadPerSec(omegaRadPerSec: Double) {}
}
