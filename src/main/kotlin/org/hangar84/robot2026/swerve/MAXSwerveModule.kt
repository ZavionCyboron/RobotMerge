package org.hangar84.robot2026.swerve

import com.revrobotics.spark.SparkBase.*
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class MAXSwerveModule(
    drivingControllerID: Int,
    turningControllerID: Int,
    private val chassisAngularOffset: Angle,
    drivingConfig: SparkMaxConfig,
    turningConfig: SparkMaxConfig,
) {
    internal val drivingController = SparkMax(drivingControllerID, MotorType.kBrushless)
    val turningController = SparkMax(turningControllerID, MotorType.kBrushless)

    var desiredState = SwerveModuleState(0.0, Rotation2d(turningController.absoluteEncoder.position))
        set(value) {
            value.angle += Rotation2d(chassisAngularOffset)

            value.optimize(Rotation2d(turningController.absoluteEncoder.position))

            val drivingStatus =
                drivingController.closedLoopController.setReference(
                    value.speedMetersPerSecond,
                    ControlType.kVelocity,
                )


            val turningStatus =
                turningController.closedLoopController.setReference(
                    value.angle.radians,
                    ControlType.kPosition,
                )

            field = value
        }

    val state
        get() =
            SwerveModuleState(
                drivingController.encoder.velocity,
                Rotation2d(Degrees.of(turningController.absoluteEncoder.position * 360) - chassisAngularOffset),
            )

    val position
        get() =
            SwerveModulePosition(
                drivingController.encoder.position,
                Rotation2d(Degrees.of(turningController.absoluteEncoder.position * 360) - chassisAngularOffset),
            )

    init {
        drivingController.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        turningController.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        drivingController.encoder.position = 0.0
    }
}