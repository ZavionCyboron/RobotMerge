package org.hangar84.robot2026.swerve

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Angle

class MAXSwerveModule(
    drivingControllerID: Int,
    turningControllerID: Int,
    private val chassisAngularOffset: Angle,
    drivingConfig: SparkMaxConfig,
    turningConfig: SparkMaxConfig,
) {
    internal val drivingController = SparkMax(drivingControllerID, MotorType.kBrushless)
    val turningController = SparkMax(turningControllerID, MotorType.kBrushless)


    private fun moduleAngle(): Rotation2d {
        // absoluteEncoder.position is already radians due to positionConversionFactor(TURNING_FACTOR)
        val encoderAngle = Rotation2d(turningController.absoluteEncoder.position)
        return encoderAngle - Rotation2d(chassisAngularOffset)
    }

    var desiredState = SwerveModuleState(0.0, moduleAngle())
        set(desired) {
            val optimized = SwerveModuleState(desired.speedMetersPerSecond, desired.angle)
            optimized.optimize(moduleAngle())

            drivingController.closedLoopController.setSetpoint(
                optimized.speedMetersPerSecond,
                ControlType.kVelocity
            )

            // Convert module target back into encoder frame for the turning controller
            val raw = (optimized.angle + Rotation2d(chassisAngularOffset)).radians
            val wrapped =
                 MathUtil.inputModulus(raw, 0.0, 2 * Math.PI)

            turningController.closedLoopController.setSetpoint(
                wrapped,
                ControlType.kPosition
            )

            field = optimized
        }

    val state: SwerveModuleState
        get() = SwerveModuleState(drivingController.encoder.velocity, moduleAngle())

    val position: SwerveModulePosition
        get() = SwerveModulePosition(drivingController.encoder.position, moduleAngle())

    init {
        drivingController.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        turningController.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        drivingController.encoder.position = 0.0
    }

    fun stop() {
        drivingController.stopMotor()
        turningController.stopMotor()
    }
}