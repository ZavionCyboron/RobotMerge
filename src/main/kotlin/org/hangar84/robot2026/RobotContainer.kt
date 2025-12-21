package org.hangar84.robot2026

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.commands.driveCommand
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.subsystems.Drivetrain
import org.hangar84.robot2026.subsystems.LauncherSubsystem
import org.hangar84.robot2026.subsystems.MecanumDriveSubsystem
import org.hangar84.robot2026.subsystems.SwerveDriveSubsystem
import edu.wpi.first.units.Units.MetersPerSecond as MPS
import edu.wpi.first.units.Units.RadiansPerSecond as RPS


object RobotContainer {

    private val MAX_SPEED = SwerveDriveSubsystem.MAX_SPEED
    private val MAX_ANGULAR_SPEED = SwerveDriveSubsystem.MAX_ANGULAR_SPEED
    private const val DRIVEDEADBAND = 0.08
    private val controller: CommandXboxController = CommandXboxController(0)
    private val buttonA = DigitalInput(19)

    val robotType: RobotType =
        if (!buttonA.get()) {
        RobotType.SWERVE
    } else {
        RobotType.MECANUM
    }

    // The robot's subsystems
    private val drivetrain: Drivetrain = when (robotType) {
        RobotType.SWERVE -> SwerveDriveSubsystem()
        RobotType.MECANUM -> MecanumDriveSubsystem()
    }
    // The driver's controller

    private val autoChooser: SendableChooser<Command> = drivetrain.buildAutoChooser()



    val autonomousCommand: Command
        get() = autoChooser.selected ?: InstantCommand()

    init {
        SmartDashboard.putString("Selected Robot Type", robotType.name)
        SmartDashboard.putData("Auto Chooser", autoChooser)
        SmartDashboard.putBoolean("DS/Enabled", DriverStation.isEnabled())
        SmartDashboard.putBoolean("DS/Auto", DriverStation.isAutonomous())
        SmartDashboard.putBoolean("DS/Teleop", DriverStation.isTeleop())

        SmartDashboard.putNumber("DS/MatchTime", DriverStation.getMatchTime())

        configureBindings()

    }
    private fun configureBindings() {
        /*val x: Double = MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND)
        val y: Double = MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND)
        val rot: Double = MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND)*/

        if (robotType == RobotType.MECANUM) {
            drivetrain.defaultCommand =  driveCommand(
                drivetrain,
                { MathUtil.applyDeadband(controller.leftX, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) },
                { false }
            )
        } else if (robotType == RobotType.SWERVE){
            drivetrain.defaultCommand =
                driveCommand(
                drivetrain,
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) * MAX_SPEED.`in`(MPS)},
                { MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND) * MAX_SPEED.`in`(MPS) },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) * MAX_ANGULAR_SPEED.`in`(RPS)},
                { true }
            )
            val swerve = drivetrain as SwerveDriveSubsystem
            controller.leftBumper().whileTrue(swerve.PARK_COMMAND)
        }

        controller.leftTrigger().onTrue(LauncherSubsystem.INTAKE_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
        controller.rightTrigger().onTrue(LauncherSubsystem.LAUNCH_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
    }
}