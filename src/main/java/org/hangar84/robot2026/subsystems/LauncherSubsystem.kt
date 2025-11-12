package org.hangar84.robot2026.subsystems

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Subsystem

object LauncherSubsystem : Subsystem {
    private val leftController = SparkMax(13, MotorType.kBrushless)
    private val rightController = SparkMax(14, MotorType.kBrushless)

    // - Commands -
    internal val LAUNCH_COMMAND
        get() = runOnce { leftController.set(1.0) }

    internal val INTAKE_COMMAND
        get() = runOnce { leftController.set(-1.0) }

    internal val STOP_COMMAND
        get() = runOnce { leftController.set(0.0) }

    init {
        val rightMotorConfig = SparkMaxConfig()
        rightMotorConfig.follow(leftController)
        rightMotorConfig.inverted(true)
        rightController.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    fun throttledLaunch(throttle: Double) {
        leftController.set(throttle)
    }
}