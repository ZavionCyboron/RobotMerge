package org.hangar84.robot2026.mecanum

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
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        encoder.position = 0.0
    }
    val positionMeters: Double get() = encoder.position
    val velocityMeters: Double get() = encoder.velocity

    fun setVelocityMps(targetMps: Double) {
        motor.closedLoopController.setReference(targetMps, SparkBase.ControlType.kVelocity)
    }
    fun stop() = motor.stopMotor()
}