package org.hangar84.robot2026.subsystems.drivebases

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class Drivetrain : SubsystemBase() {

    abstract val maxLinearSpeedMps: Double
    abstract val maxAngularSpeedRadPerSec: Double

    abstract fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean)
    abstract fun buildAutoChooser(): SendableChooser<Command>
    abstract fun getChassisSpeeds(): ChassisSpeeds

    abstract fun getHeading(): Rotation2d
    abstract fun resetPose(pose: Pose2d)
    abstract fun zeroHeading()

    open fun getPose(): Pose2d = Pose2d()

    // -- Simulation --
    open fun simulationPeriodic(dtSeconds: Double) {}
}