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

    private val relEncoder = hinge_Motor.encoder
    private val gearRatio = 75

    private val positionConversionFactor = 360.0 / gearRatio
    private val velocityConversionFactor = 360.0 / gearRatio/ 60.0

    private val maxLimitSwitchOne = cfg.maxLimitSwitchOneDio?.let { DigitalInput(it) }
    private val maxLimitSwitchTwo = cfg.maxLimitSwitchTwoDio?.let { DigitalInput(it) }

    init {
        val config = SparkFlexConfig().apply {
            smartCurrentLimit(currentLimit) // 30 amps
                .encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor)
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
        relEncoder.position = targetAngleDeg
    }

    override fun updateInputs(inputs: HingeIO.Inputs) {
        inputs.angleDeg = relEncoder.position

        inputs.maxLimitSwitchOneDioPressed = maxLimitSwitchOne?.let { !it.get() } ?: false
        inputs.maxLimitSwitchTwoDioPressed = maxLimitSwitchTwo?.let { !it.get() } ?: false
    }
}