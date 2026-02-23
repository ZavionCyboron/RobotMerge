package org.hangar84.robot2026.io.real.mechanismrealio

import edu.wpi.first.wpilibj.DigitalInput
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkFlexConfig
import com.revrobotics.spark.SparkFlex
import org.hangar84.robot2026.constants.Hinge
import org.hangar84.robot2026.constants.MaxConfig
import com.revrobotics.spark.config.LimitSwitchConfig
import org.hangar84.robot2026.io.interfaces.mechanismio.HingeIO

class RevHingeIO(cfg: Hinge, maxcfg: MaxConfig): HingeIO {

    private val currentLimit = maxcfg.currentLimit

    private val hinge_Motor = SparkFlex(cfg.hingeMotorId, MotorType.kBrushless)

    private val maxLimitSwitchOne = cfg.maxLimitSwitchOneDio?.let { DigitalInput(it) }
    private val maxLimitSwitchTwo = cfg.maxLimitSwitchTwoDio?.let { DigitalInput(it) }

    private val gearRatio = 75

    init {
        val config = SparkFlexConfig().apply {
            smartCurrentLimit(currentLimit) // 30 amps
            limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        }
        hinge_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        hinge_Motor.set(percent)
    }

    override fun updateInputs(inputs: HingeIO.Inputs) {
        val motorRotations = hinge_Motor.absoluteEncoder.position

        val hingeRotations = motorRotations / gearRatio // the motors rotation divided by the gear ratio
        val hingeDegrees = hingeRotations * 360.0

        inputs.angleDeg = hingeDegrees
        inputs.maxLimitSwitchOneDioPressed = maxLimitSwitchOne?.let { !it.get() } ?: false
        inputs.maxLimitSwitchTwoDioPressed = maxLimitSwitchTwo?.let { !it.get() } ?: false
    }
}