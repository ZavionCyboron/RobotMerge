package org.hangar84.robot2026.io.real.mechanismrealio

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Intake
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.io.interfaces.mechanismio.IntakeIO

class RevIntakeIO(cfg: Intake, maxcfg: MaxConfig) : IntakeIO {
    private val currentLimit = maxcfg.currentLimit
    private val invertedTrue = maxcfg.inverted
    private val leftIntake = SparkMax(cfg.leftIntakeMotorID, MotorType.kBrushless)
    private val rightIntake = SparkMax(cfg.rightIntakeMotorID, MotorType.kBrushless)

    init {
        val leftconfig = SparkMaxConfig().apply {
            smartCurrentLimit(currentLimit) // 30 amps
                .inverted(invertedTrue) // true
        }
        val rightConfig = SparkMaxConfig().apply{
            smartCurrentLimit(currentLimit)
        }
        leftIntake.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightIntake.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

    }

    override fun setPercent(percent: Double) {
        leftIntake.set(percent)
        rightIntake.set(percent)
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.leftAppliedOutput = leftIntake.appliedOutput
        inputs.leftCurrentAmps = leftIntake.outputCurrent
        inputs.leftTempCelcius = leftIntake.motorTemperature
    }
}