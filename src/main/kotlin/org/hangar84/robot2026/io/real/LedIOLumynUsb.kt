package org.hangar84.robot2026.io.real

import com.lumynlabs.connection.usb.USBPort
import com.lumynlabs.devices.ConnectorXAnimate
import com.lumynlabs.domain.led.Animation
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.util.Color
import org.hangar84.robot2026.io.LedIO

class LedIOLumynUsb(
    private val usbPort: USBPort = USBPort.kUSB1,
    private val groupId: String = "all"
) : LedIO {

    private val cx = ConnectorXAnimate()
    private var connected = false
    private var lasCmdSig: String? = null

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

    override fun setSolid(color: Color) = sendOnce("solid:${color.red}:${color.green}:${color.blue}") {
        cx.leds.SetGroupColor(groupId, color)
    }

    override fun setOff() = setSolid(Color(0.0, 0.0, 0.0))

    override fun setBreathe(color: Color, periodMs: Int) {
        sendOnce("breathe:${color.red}:${color.green}:${color.blue}:$periodMs") {
            cx.leds.SetAnimation(Animation.Breathe)
                .ForGroup(groupId)
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(periodMs.toDouble()))
                .Reverse(false)
                .RunOnce(false)
        }
    }

    override fun setChase(color: Color, speedMs: Int, reverse: Boolean) {
        sendOnce("chase:${color.red}:${color.green}:${color.blue}:$speedMs:$reverse") {
            cx.leds.SetAnimation(Animation.Chase)
                .ForGroup(groupId)
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(speedMs.toDouble()))
                .Reverse(reverse)
                .RunOnce(false)
        }
    }

    override fun setStrobe(color: Color, speedMs: Int) {
        sendOnce("strobe:${color.red}:${color.green}:${color.blue}:$speedMs") {
            cx.leds.SetAnimation(Animation.Blink)
                .ForGroup(groupId)
                .WithColor(color)
                .WithDelay(Units.Milliseconds.of(speedMs.toDouble()))
                .Reverse(false)
                .RunOnce(false)
        }
    }
}