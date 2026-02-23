package org.hangar84.robot2026.io.real.mechanismrealio

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkFlexConfig
import edu.wpi.first.wpilibj.DigitalInput
import org.hangar84.robot2026.constants.Hinge
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.io.interfaces.mechanismio.HingeIO

class RevHingeIO(cfg: Hinge, maxcfg: MaxConfig): HingeIO {

    private val currentLimit = maxcfg.currentLimit

    private val hinge_Motor = SparkFlex(cfg.hingeMotorId, MotorType.kBrushless)

    private val maxLimitSwitchOne = cfg.maxLimitSwitchOneDio?.let { DigitalInput(it) }
    private val maxLimitSwitchTwo = cfg.maxLimitSwitchTwoDio?.let { DigitalInput(it) }

    init {
        val config = SparkFlexConfig().apply {
            smartCurrentLimit(currentLimit) // 30 amps
        }
        hinge_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        hinge_Motor.set(percent)
    }

    private var zeroOffsetDeg = 0.0  // calibrate this once

    private fun wrap0to360(deg: Double): Double {
        var x = deg % 180.0
        if (x < 0) x += 180.0
        return x
    }

    override fun calibrateAbsoluteTo(targetAngleDeg: Double) {
        val absRot = hinge_Motor.absoluteEncoder.position
        val rawDeg = absRot * 360.0
        zeroOffsetDeg = wrap0to360(rawDeg - targetAngleDeg)
    }

    override fun updateInputs(inputs: HingeIO.Inputs) {
        val absRot = hinge_Motor.absoluteEncoder.position   // typically 0..1
        val rawDeg = absRot * 360.0
        inputs.angleDeg = wrap0to360(rawDeg - zeroOffsetDeg)

        inputs.maxLimitSwitchOneDioPressed = maxLimitSwitchOne?.let { !it.get() } ?: false
        inputs.maxLimitSwitchTwoDioPressed = maxLimitSwitchTwo?.let { !it.get() } ?: false
    }
}