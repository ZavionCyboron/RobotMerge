package org.hangar84.robot2026.commands

import edu.wpi.first.wpilibj2.command.RunCommand
import org.hangar84.robot2026.subsystems.drivebases.Drivetrain

fun driveCommand(
    drivetrain: Drivetrain,
    xSupplier: () -> Double,
    ySupplier: () -> Double,
    rotSupplier: () -> Double,
    fieldRelative: () -> Boolean
) = RunCommand(
    {
        drivetrain.drive(
            xSupplier(),
            ySupplier(),
            rotSupplier(),
            fieldRelative()
        )
    },
    drivetrain
)