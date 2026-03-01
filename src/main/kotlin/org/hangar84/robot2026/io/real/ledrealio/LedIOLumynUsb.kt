package org.hangar84.robot2026.io.real.ledrealio

import com.lumynlabs.connection.usb.USBPort
import com.lumynlabs.devices.ConnectorXAnimate
import com.lumynlabs.domain.led.Animation
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.util.Color
import org.hangar84.robot2026.io.interfaces.ledio.LedIO
import org.hangar84.robot2026.io.interfaces.ledio.LedTarget

class LedIOLumynUsb(
    private val usbPort: USBPort = USBPort.kUSB1,
    private val groupBase: String = "base",
    private val groupIntake: String = "intake",
    private val groupLauncher: String = "launcher",
    private val groupALL: String = "all",
) : LedIO {

    private val cx = ConnectorXAnimate()
    private var connected = false
    private var lasCmdSig: String? = null

    private fun groupId(t: LedTarget) = when (t) {
        LedTarget.BASE -> groupBase
        LedTarget.INTAKE ->  groupIntake
        LedTarget.LAUNCHER -> groupLauncher
        LedTarget.ALL -> groupALL
    }

    override fun connect() {
        connected = cx.Connect(usbPort)
    }

    override fun updateInputs(inputs: LedIO.Inputs) {
        inputs.connected = connected
    }

    private fun sendOnce(signature: String, send: () -> Unit) {
        if (!connected) return
        if (signature == lasCmdSig) return
        lasCmdSig = signature
        send()
    }

    override fun setSolid(target: LedTarget, color: Color) = sendOnce("solid:${color.red}:${color.green}:${color.blue}") {
        cx.leds.SetGroupColor(groupId(target), color)
    }

    override fun setOff() = setSolid(LedTarget.BASE, Color(0.0, 0.0, 0.0))

    override fun setBreathe(target: LedTarget, color: Color, periodMs: Int) {
        sendOnce("breathe:${color.red}:${color.green}:${color.blue}:$periodMs") {
            cx.leds.SetAnimation(Animation.Breathe)
                .ForGroup(groupId(target))
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(periodMs.toDouble()))
                .Reverse(false)
                .RunOnce(false)
        }
    }

    override fun setChase(target: LedTarget, color: Color, speedMs: Int, reverse: Boolean) {
        sendOnce("chase:${color.red}:${color.green}:${color.blue}:$speedMs:$reverse") {
            cx.leds.SetAnimation(Animation.Chase)
                .ForGroup(groupId(target))
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(speedMs.toDouble()))
                .Reverse(reverse)
                .RunOnce(false)
        }
    }

    override fun setStrobe(target: LedTarget, color: Color, speedMs: Int) {
        sendOnce("strobe:${color.red}:${color.green}:${color.blue}:$speedMs") {
            cx.leds.SetAnimation(Animation.Blink)
                .ForGroup(groupId(target))
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(speedMs.toDouble()))
                .Reverse(false)
                .RunOnce(false)
        }
    }
}