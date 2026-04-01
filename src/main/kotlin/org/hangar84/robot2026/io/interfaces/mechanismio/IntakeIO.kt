package org.hangar84.robot2026.io.interfaces.mechanismio

interface IntakeIO {
    data class Inputs(
        var leftAppliedOutput: Double = 0.0,
        var rightAppliedOutput: Double = 0.0,
        var leftTempCelcius: Double = 0.0,
        var rightTempCelcius: Double = 0.0,
        var leftCurrentAmps: Double = 0.0,
        var rightCurrentAmps: Double = 0.0,
    )

    fun updateInputs(inputs: Inputs) {}
    fun setPercent(percent: Double) {}
    fun stop() = setPercent(0.0)
}