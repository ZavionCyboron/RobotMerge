package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Constants.Intake
import org.hangar84.robot2026.io.IntakeIO

class RevIntakeIO : IntakeIO {
    private val leftIntake = SparkMax(Intake.Left_Intake_Motor, MotorType.kBrushless)
    private val rightIntake = SparkMax(Intake.Right_Intake_Motor, MotorType.kBrushless)

    init {
        val leftconfig = SparkMaxConfig().apply {
            smartCurrentLimit(30)
                .inverted(true)
        }
        val rightConfig = SparkMaxConfig().apply{
            smartCurrentLimit(30)
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