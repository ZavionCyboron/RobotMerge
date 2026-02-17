package org.hangar84.robot2026.io.interfaces.ledio

import edu.wpi.first.wpilibj.util.Color

class LedIOMulti(private vararg val ios: LedIO) : LedIO {
    override fun connect() = ios.forEach { it.connect() }

    override fun updateInputs(inputs: LedIO.Inputs) {
        ios.forEach { it.updateInputs(inputs) }
    }

    override fun setSolid(target: LedTarget, color: Color) =
        ios.forEach { it.setSolid(target, color) }

    override fun setBreathe(target: LedTarget, color: Color, periodMs: Int) =
        ios.forEach { it.setBreathe(target, color, periodMs) }

    override fun setChase(target: LedTarget, color: Color, speedMs: Int, reverse: Boolean) =
        ios.forEach { it.setChase(target, color, speedMs, reverse) }

    override fun setStrobe(target: LedTarget, color: Color, speedMs: Int) =
        ios.forEach { it.setStrobe(target, color, speedMs) }

    override fun setOff() = ios.forEach { it.setOff() }
}