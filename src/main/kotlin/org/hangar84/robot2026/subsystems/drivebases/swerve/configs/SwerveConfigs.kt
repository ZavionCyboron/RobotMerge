package org.hangar84.robot2026.subsystems.drivebases.swerve.configs

import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig

data object SwerveConfigs {
    private const val DRIVE_PINION_TEETH = 13
    private const val WHEEL_DIAMETER = 0.0762
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
    private const val DRIVING_REDUCTION = (45 * 22) / (DRIVE_PINION_TEETH * 15)

    private const val DRIVING_FACTOR = WHEEL_CIRCUMFERENCE / DRIVING_REDUCTION
    private const val TURNING_FACTOR = 2 * Math.PI

    val drivingConfig = SparkMaxConfig()
    val turningConfig = SparkMaxConfig()

    init {
        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)

        drivingConfig.encoder
            .positionConversionFactor(DRIVING_FACTOR)
            .velocityConversionFactor(DRIVING_FACTOR / 60)

        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0.0, 0.0) // TODO: Tune these values (if needed)
            .outputRange(-1.0, 1.0)

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)

        turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(TURNING_FACTOR)
            .velocityConversionFactor(TURNING_FACTOR / 60)

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0.0, 0.0) // TODO: Tune these values (if needed)
            .outputRange(-1.0, 1.0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, TURNING_FACTOR)
    }
}
