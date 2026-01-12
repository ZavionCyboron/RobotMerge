package org.hangar84.robot2026.mecanum

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig

class MecanumModule(
    val name: String,
    canID: Int,
    motorConfig: SparkMaxConfig,
) {
    val motor = SparkMax(canID, SparkLowLevel.MotorType.kBrushless)
    val encoder = motor.encoder

    init {
        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        encoder.position = 0.0
    }
    val positionMeters: Double get() = encoder.position
    val velocityMeters: Double get() = encoder.velocity

    fun setVelocityMps(targetMps: Double) {
        motor.closedLoopController
        motor.closedLoopController.setSetpoint(targetMps, SparkBase.ControlType.kVelocity)
    }
    fun stop() = motor.stopMotor()
}