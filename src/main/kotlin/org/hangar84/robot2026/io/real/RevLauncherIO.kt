package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Constants.Launcher
import org.hangar84.robot2026.io.LauncherIO


class RevLauncherIO : LauncherIO {
    private val leftLaunch = SparkMax(Launcher.Launcher_Left_Motor, MotorType.kBrushless)
    private val rightLaunch = SparkMax(Launcher.Launcher_Right_Motor, MotorType.kBrushless)

    init {
        val rightCfg = SparkMaxConfig().apply {
            smartCurrentLimit(50)
                .inverted(true)

        }
        val leftCfg = SparkMaxConfig().apply {
            smartCurrentLimit(50)
        }
        leftLaunch.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightLaunch.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setLeftPercent(percent: Double) {
        leftLaunch.set(percent)
    }

    override fun setRightPercent(percent: Double) {
        rightLaunch.set(percent)
    }

    override fun setPercent(percent: Double) {
        setRightPercent(percent)
        setLeftPercent(percent)
    }

    override fun updateInputs(inputs: LauncherIO.Inputs) {
        inputs.leftAppliedOutput = leftLaunch.appliedOutput
        inputs.leftCurrentAmps = leftLaunch.outputCurrent
        inputs.leftTempCelsius = leftLaunch.motorTemperature

        inputs.rightAppliedOutput = rightLaunch.appliedOutput
        inputs.rightCurrentAmps = rightLaunch.outputCurrent
        inputs.rightTempCelsius = rightLaunch.motorTemperature
    }
}