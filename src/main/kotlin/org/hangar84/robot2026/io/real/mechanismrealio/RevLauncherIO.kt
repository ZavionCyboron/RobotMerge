package org.hangar84.robot2026.io.real.mechanismrealio

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.io.interfaces.mechanismio.LauncherIO
import org.hangar84.robot2026.constants.Launcher
import org.hangar84.robot2026.constants.MaxConfig


class RevLauncherIO(cfg: Launcher, maxcfg: MaxConfig) : LauncherIO {
    private val leftLaunch = SparkMax(cfg.leftLauncherId, MotorType.kBrushless)
    private val rightLaunch = SparkMax(cfg.rightLauncherId, MotorType.kBrushless)

    private val invertedTrue = maxcfg.inverted
    private val currentLimit = maxcfg.currentLimit + 10

    init {
        val rightCfg = SparkMaxConfig().apply {
            smartCurrentLimit(currentLimit) // 40 amps
                .inverted(invertedTrue) // True

        }
        val leftCfg = SparkMaxConfig().apply {
            smartCurrentLimit(currentLimit) // 40 amps
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