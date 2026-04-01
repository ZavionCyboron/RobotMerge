package org.hangar84.robot2026.io.interfaces.mechanismio

interface HingeIO {
    data class Inputs(
        var angleDeg: Double = 0.0,
        var maxLimitSwitchOneDioPressed: Boolean = false,
        var maxLimitSwitchTwoDioPressed: Boolean = false
    )

    fun updateInputs(inputs: Inputs)

    fun calibrateAbsoluteTo(targetAngleDeg: Double) {}

    fun setPercent(percent: Double) {}

    fun stop() = setPercent(0.0)
}