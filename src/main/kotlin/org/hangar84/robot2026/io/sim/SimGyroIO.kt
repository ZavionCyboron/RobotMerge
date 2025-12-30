package org.hangar84.robot2026.io.sim

import edu.wpi.first.math.geometry.Rotation2d
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.sim.SimSensors

class SimGyroIO : GyroIO {
    private var yaw = Rotation2d()
    private var yawRateDegPerSec = 0.0
    private var omegaRadPerSec = 0.0

    override fun setSimOmegaRadPerSec(omegaRadPerSec: Double) {
        this.omegaRadPerSec = omegaRadPerSec
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        yaw = yaw.plus(Rotation2d(omegaRadPerSec * dtSeconds))
        yawRateDegPerSec = Math.toDegrees(omegaRadPerSec)
    }

    override fun updateInputs(inputs: GyroIO.Inputs) {
        inputs.yaw = yaw
        inputs.yawRateDegPerSec = yawRateDegPerSec
    }

    override fun zeroYaw() {
        yaw = Rotation2d()
    }
}