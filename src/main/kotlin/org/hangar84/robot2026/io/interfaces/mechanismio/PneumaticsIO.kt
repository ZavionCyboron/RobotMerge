package org.hangar84.robot2026.io.interfaces.mechanismio

interface PneumaticsIO {
    enum class State { EXTEND, //Forward
         RETRACT, //Reverse
        NEUTRAL  // Off
        }

    data class Inputs(
        var Left: State = State.NEUTRAL,
        var Right: State = State.NEUTRAL,
        var CompressorEnabled: Boolean = false,
        var Left_Solenoid_Extend: Boolean = false,
        var Left_Solenoid_Retract: Boolean = false,
        var Right_Solenoid_Extend: Boolean = false,
        var Right_Solenoid_Retract: Boolean = false,
    )

    fun updateInputs(inputs: Inputs)

    fun Left(state: State)
    fun Right(state: State)

    fun setCompressor(enabled: Boolean)
}