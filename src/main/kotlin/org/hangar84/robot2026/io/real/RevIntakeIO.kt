package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.io.IntakeIO
import org.hangar84.robot2026.constants.Constants.Intake

class RevIntakeIO : IntakeIO {
    private val leftIntake = SparkMax(Intake.Intake_Motor, MotorType.kBrushed)

    init {
        val config = SparkMaxConfig().apply {
            smartCurrentLimit(20)
        }
        leftIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        leftIntake.set(percent)
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.leftAppliedOutput = leftIntake.appliedOutput
        inputs.leftCurrentAmps = leftIntake.outputCurrent
        inputs.leftTempCelcius = leftIntake.motorTemperature
    }
}