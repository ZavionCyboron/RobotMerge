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
        leftIntake.configure(
            SparkMaxConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    override fun setPercent(percent: Double) {
        leftIntake.set(percent)
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.leftCurrentAmps = leftIntake.outputCurrent

        val busVoltage = leftIntake.busVoltage
        val motorVoltage = inputs.leftAppliedOutput * busVoltage

        SmartDashboard.putNumber("Intake AppliedVoltage", motorVoltage)
        SmartDashboard.putNumber("Intake CurrentAmps", inputs.leftCurrentAmps)
        SmartDashboard.putNumber("Intake TemperatureCelsius", leftIntake.motorTemperature)
    }
}