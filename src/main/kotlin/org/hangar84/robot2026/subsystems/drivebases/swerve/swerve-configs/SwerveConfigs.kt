package org.hangar84.robot2026.subsystems.drivebases.swerve.`swerve-configs`

import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig


data object SwerveConfigs {

    private const val DRIVE_PINION_TEETH = 13
    private const val NEO_FREE_SPEED_RPM = 5676
    private const val WHEEL_DIAMETER = 0.0762
    private const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI
    private const val DRIVING_REDUCTION = (45 * 22) / (DRIVE_PINION_TEETH * 15)
    private const val DRIVE_WHEEL_FREE_SPEED_RPS = (NEO_FREE_SPEED_RPM / 60) * WHEEL_CIRCUMFERENCE / DRIVING_REDUCTION

    private const val DRIVING_FACTOR = WHEEL_CIRCUMFERENCE / DRIVING_REDUCTION
    private const val TURNING_FACTOR = 2 * Math.PI
    private const val DRIVING_FEEDFORWARD = 1.0 / DRIVE_WHEEL_FREE_SPEED_RPS


    val drivingConfig: SparkMaxConfig = SparkMaxConfig()
    val turningConfig: SparkMaxConfig = SparkMaxConfig()

    init {

        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
        drivingConfig.encoder
            .positionConversionFactor(DRIVING_FACTOR) // meters
            .velocityConversionFactor(DRIVING_FACTOR / 60.0) // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // These are example gains you may need to them for your own robot!
            .pid(0.04, 0.0, 0.0)
            .outputRange(-1.0, 1.0)

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
        turningConfig.absoluteEncoder // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(TURNING_FACTOR) // radians
            .velocityConversionFactor(TURNING_FACTOR / 60.0) // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // These are example gains you may need to them for your own robot!
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0) // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, TURNING_FACTOR)
    }
}