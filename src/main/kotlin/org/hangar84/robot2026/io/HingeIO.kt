package org.hangar84.robot2026.io

interface HingeIO {
    data class Inputs(
        var angleDeg: Double = 0.0
    )

    fun updateInputs(inputs: Inputs)

    fun setPercent(percent: Double) {}

    fun stop() = setPercent(0.0)
}