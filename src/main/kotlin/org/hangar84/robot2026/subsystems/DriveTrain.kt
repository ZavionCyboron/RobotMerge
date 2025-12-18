package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class Drivetrain : SubsystemBase() {
    abstract fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean)
    abstract fun buildAutoChooser(): SendableChooser<Command>
}