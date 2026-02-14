package org.hangar84.robot2026.subsystems.drivebases.mecanum.`mecanum-configs`

import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig

object MecanumConfigs {
    private const val WHEEL_DIAMETER_M = 0.14732
    private const val WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI
    private const val GEAR_RATIO = 4.5

    private const val  POSITION_FACTOR_M = WHEEL_CIRCUMFERENCE_M / GEAR_RATIO
    private const val VELOCITY_FACTOR_MPS = POSITION_FACTOR_M / 60.0

    val driveConfig: SparkMaxConfig = SparkMaxConfig()

    init {
        driveConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
        driveConfig.encoder
            .positionConversionFactor(POSITION_FACTOR_M)
            .velocityConversionFactor(VELOCITY_FACTOR_MPS)
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0002, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
    }
}