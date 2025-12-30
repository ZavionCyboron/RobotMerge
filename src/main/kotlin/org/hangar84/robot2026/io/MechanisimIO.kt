package org.hangar84.robot2026.io

interface MechanisimIO {

    data class Inputs(
        var leftAppliedOutput: Double = 0.0,
        var rightAppliedOutput: Double = 0.0,

        var leftVelocityRpm: Double = 0.0,
        var rightVelocityRpm: Double = 0.0,

        var leftCurrentAmps: Double = 0.0,
        var rightCurrentAmps: Double = 0.0
    )

    fun updateInputs(inputs: Inputs) {}

    /** Percent output [-1..1] */
    fun setPercent(percent: Double) {}

    fun stop() {
        setPercent(0.0)
    }

    /** Optional sim hook */
    fun simulationPeriodic(dtSeconds: Double) {}
}