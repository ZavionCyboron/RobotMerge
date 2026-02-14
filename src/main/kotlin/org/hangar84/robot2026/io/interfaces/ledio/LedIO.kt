package org.hangar84.robot2026.io.interfaces.ledio

import edu.wpi.first.wpilibj.util.Color


enum class LedTarget { BASE, INTAKE, LAUNCHER, NULL }

interface LedIO {
    data class Inputs(
        var connected: Boolean = false,
        var lastError: String = ""
    )

    fun updateInputs(inputs: Inputs) {}

    fun connect() {}

    fun setSolid(target: LedTarget, color: Color) {}

    fun setBreathe(target: LedTarget, color: Color, periodMs: Int) {}

    fun setChase(target: LedTarget, color: Color, speedMs: Int, reverse: Boolean = false) {}

    fun setStrobe(target: LedTarget, color: Color, speedMs: Int) {}

    fun setOff() {}
}