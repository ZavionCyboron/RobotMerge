package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.constants.Constants.Launcher
import org.hangar84.robot2026.io.LauncherIO


class RevLauncherIO : LauncherIO {
    private val leftLaunch = SparkMax(Launcher.Left_Launcher_Motor_ID, MotorType.kBrushed)
    private val rightLaunch = SparkMax(Launcher.Right_Launcher_Motor_ID, MotorType.kBrushed)

    init {
        val rightCfg = SparkMaxConfig().apply {
        }
        val leftCfg = SparkMaxConfig().apply {
            inverted(true)
        }
        leftLaunch.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightLaunch.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        leftLaunch.set(percent)
        rightLaunch.set(percent)
    }

    override fun updateInputs(inputs: LauncherIO.Inputs) {

        val leftLauncherMotorVolts = leftLaunch.busVoltage
        val rightLauncherMotorVolts = rightLaunch.busVoltage

        val leftLauncherMotorAppliedVolts = inputs.leftAppliedOutput * leftLauncherMotorVolts
        val rightLauncherMotorAppliedVolts = inputs.rightAppliedOutput * rightLauncherMotorVolts

        SmartDashboard.putNumber("Launcher Left CurrentAmps", inputs.leftCurrentAmps)
        SmartDashboard.putNumber("Launcher Left AppliedVolts", leftLauncherMotorAppliedVolts)
        SmartDashboard.putNumber("Launcher Left TempCelsius", leftLaunch.motorTemperature)

        SmartDashboard.putNumber("Launcher Right CurrentAmps", inputs.rightCurrentAmps)
        SmartDashboard.putNumber("Launcher Right AppliedVolts", rightLauncherMotorAppliedVolts)
        SmartDashboard.putNumber("Launcher Right TempCelsius", rightLaunch.motorTemperature)

        SmartDashboard.putNumber("SystemBusVoltage", leftLauncherMotorVolts)
    }
}