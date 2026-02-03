package org.hangar84.robot2026.io

interface LauncherIO {
    data class Inputs(
        var leftAppliedOutput: Double = 0.0,
        var rightAppliedOutput: Double = 0.0,
        var leftCurrentAmps: Double = 0.0,
        var rightCurrentAmps: Double = 0.0,
        var leftTempCelsius: Double = 0.0,
        var rightTempCelsius: Double = 0.0
    )

    fun updateInputs(inputs: Inputs) {}
    fun setPercent(percent: Double) {}
    fun stop() = setPercent(0.0)
    fun simulationPeriodic(dtSeconds: Double) {}
}