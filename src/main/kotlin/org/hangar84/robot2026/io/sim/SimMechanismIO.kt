package org.hangar84.robot2026.io.sim

import org.hangar84.robot2026.io.MechanisimIO
import kotlin.math.abs

class SimMechanismIO : MechanisimIO {

    private var percent = 0.0
    private var rpm = 0.0

    override fun setPercent(percent: Double) {
        this.percent = percent.coerceIn(-1.0, 1.0)
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        // Simple first-order response (tweakable)
        val freeSpeedRpm = 5000.0
        val target = percent * freeSpeedRpm

        val tau = 0.15
        rpm += (target - rpm) * (dtSeconds / tau).coerceIn(0.0, 1.0)
    }

    override fun updateInputs(inputs: MechanisimIO.Inputs) {
        inputs.leftAppliedOutput = percent
        inputs.rightAppliedOutput = percent

        inputs.leftVelocityRpm = rpm
        inputs.rightVelocityRpm = rpm

        // Fake current draw
        inputs.leftCurrentAmps = 10.0 * abs(percent)
        inputs.rightCurrentAmps = 10.0 * abs(percent)
    }
}