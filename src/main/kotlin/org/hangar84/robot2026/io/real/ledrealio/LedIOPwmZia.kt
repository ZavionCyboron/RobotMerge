package org.hangar84.robot2026.io.real.ledrealio

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import org.hangar84.robot2026.io.interfaces.ledio.LedIO
import org.hangar84.robot2026.io.interfaces.ledio.LedTarget

class LedIOPwmZia(pwmPort: Int = 0) : LedIO {
    private val led = AddressableLED(pwmPort)
    private val buffer = AddressableLEDBuffer(48) // Total Zia Pixels

    init {
        led.setLength(48)
        led.setData(buffer)
        led.start()
    }

    private fun updateAll(color: Color) {
        for (i in 0 until 48) {
            buffer.setLED(i, color)
        }
        led.setData(buffer)
    }

    override fun setStrobe(target: LedTarget, color: Color, speedMs: Int) {
        if (target == LedTarget.ALL) {
            val on = (Timer.getFPGATimestamp() * 1000 / speedMs).toInt() % 2 == 0
            updateAll(if (on) color else Color.kBlack)
        } else {
            updateAll(Color.kBlack)
        }
    }

    override fun setChase(target: LedTarget, color: Color, speedMs: Int, reverse: Boolean) {
        // DS Disconnected fallback
        if (target == LedTarget.ALL) updateAll(color) else updateAll(Color.kBlack)
    }

    override fun setBreathe(target: LedTarget, color: Color, periodMs: Int) {
        if (target == LedTarget.ALL) {
            val luma = (Math.sin(Timer.getFPGATimestamp() * 1000 * 2 * Math.PI / periodMs) + 1.0) / 2.0
            updateAll(Color(color.red * luma, color.green * luma, color.blue * luma))
        } else {
            updateAll(Color.kBlack)
        }
    }

    override fun setOff() = updateAll(Color.kBlack)
}