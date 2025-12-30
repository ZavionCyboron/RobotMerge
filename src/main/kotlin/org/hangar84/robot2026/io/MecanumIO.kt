package org.hangar84.robot2026.io



interface MecanumIO {
    data class Inputs(
        var flPosMeters: Double = 0.0,
        var frPosMeters: Double = 0.0,
        var rlPosMeters: Double = 0.0,
        var rrPosMeters: Double = 0.0,

        var flVelMps: Double = 0.0,
        var frVelMps: Double = 0.0,
        var rlVelMps: Double = 0.0,
        var rrVelMps: Double = 0.0
    )

    fun updateInputs(inputs: Inputs)

    fun setWheelSpeeds(
        fl: Double,
        fr: Double,
        rl: Double,
        rr: Double
    )

    fun stop()

    fun simulationPeriodic(dtSeconds: Double) {}
}