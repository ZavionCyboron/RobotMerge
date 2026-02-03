package org.hangar84.robot2026.io

interface IntakeIO {
    data class Inputs(
        var leftAppliedOutput: Double = 0.0,
        var leftTempCelcius: Double = 0.0,
        var leftCurrentAmps: Double = 0.0,
    )

    fun updateInputs(inputs: Inputs) {}
    fun setPercent(percent: Double) {}
    fun stop() = setPercent(0.0)
}