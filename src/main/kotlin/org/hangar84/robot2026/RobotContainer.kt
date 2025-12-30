package org.hangar84.robot2026

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.commands.driveCommand
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.io.SwerveIO
import org.hangar84.robot2026.io.real.AdisGyroIO
import org.hangar84.robot2026.io.real.MaxSwerveIO
import org.hangar84.robot2026.io.real.RevMecanumIO
import org.hangar84.robot2026.io.real.RevMechanisimIO
import org.hangar84.robot2026.io.sim.SimGyroIO
import org.hangar84.robot2026.io.sim.SimMecanumIO
import org.hangar84.robot2026.io.sim.SimMechanismIO
import org.hangar84.robot2026.io.sim.SimSwerveIO
import org.hangar84.robot2026.sim.*
import org.hangar84.robot2026.sim.SimClock.dtSeconds
import org.hangar84.robot2026.sim.SimField.publishOnce
import org.hangar84.robot2026.sim.SimField.setRobotPose
import org.hangar84.robot2026.sim.SimState.isSim
import org.hangar84.robot2026.subsystems.Drivetrain
import org.hangar84.robot2026.subsystems.LauncherSubsystem
import org.hangar84.robot2026.subsystems.MecanumDriveSubsystem
import org.hangar84.robot2026.subsystems.SwerveDriveSubsystem
import org.hangar84.robot2026.telemetry.TelemetryRouter
import kotlin.math.withSign


object RobotContainer {
    private const val DRIVEDEADBAND = 0.08
    private val controller: CommandXboxController = CommandXboxController(0)
    private val buttonA = DigitalInput(19)

    val launcher = LauncherSubsystem(
        if (RobotBase.isSimulation()) SimMechanismIO() else RevMechanisimIO()
    )

    private fun readBootSelector(): RobotType {
        return if (!buttonA.get()) {
            RobotType.SWERVE
        } else {
            RobotType.MECANUM
        }
    }

    val robotType: RobotType =
        if (RobotBase.isSimulation()) {
            SimRobotTypeSelector.selected()
        } else {
            readBootSelector()
        }

    // The robot's subsystems
    val drivetrain: Drivetrain = when (robotType) {
        RobotType.SWERVE -> {
            val gyro: GyroIO = if (isSim) SimGyroIO() else AdisGyroIO()
            val swerve: SwerveIO = if (isSim) SimSwerveIO() else MaxSwerveIO()
            SwerveDriveSubsystem(swerve, gyro)
        }

        RobotType.MECANUM -> {
            val gyro: GyroIO = if (isSim) SimGyroIO() else AdisGyroIO()
            val mecanum: MecanumIO = if (isSim) SimMecanumIO() else RevMecanumIO()
            MecanumDriveSubsystem(mecanum, gyro)
        }
    }
    // The driver's controller

    private val autoChooser: SendableChooser<Command> = drivetrain.buildAutoChooser()



    val autonomousCommand: Command
        get() = autoChooser.selected ?: InstantCommand()


    init {
        TelemetryRouter.setBase(
            if (isSim) {
                "${robotType.name}/Sim"
            } else {
                robotType.name
            }
        )

        if (isSim) {
            // ***
            // make sure to uncomment the one you want to use
            // and to comment the one that was previously used so there is no conflictions
            // ***

            SimField.leftBluePose()

            //SimField.middleBluePose()

            //SimField.rightBluePose()

            //SimField.rightRedPose()

            //SimField.middleRedPose()

            //SimField.leftRedPose()
        }

        SmartDashboard.putString("Selected Robot Type", robotType.name)
        SmartDashboard.putData("Auto Chooser", autoChooser)

        SmartDashboard.putBoolean("DS/Enabled", DriverStation.isEnabled())
        SmartDashboard.putBoolean("DS/Auto", DriverStation.isAutonomous())
        SmartDashboard.putBoolean("DS/Teleop", DriverStation.isTeleop())

        SmartDashboard.putNumber("DS/MatchTime", DriverStation.getMatchTime())

        setupDashboard()
        SimHooks.init()
        configureBindings()

    }
    private fun configureBindings() {

        val xLimiter = SlewRateLimiter(5.0)   // m/s^2 style feel
        val yLimiter = SlewRateLimiter(5.0)
        val rotLimiter = SlewRateLimiter(2.0) // rad/s^2 feel (or deg/s^2)

        fun shapedAxis(raw: Double): Double {
            val db = MathUtil.applyDeadband(raw, DRIVEDEADBAND)
            return (db * db).withSign(db) // square for finer control near center
        }

        if (robotType == RobotType.MECANUM) {
            drivetrain.defaultCommand =  driveCommand(
                drivetrain,
                { xLimiter.calculate(shapedAxis(-controller.leftY)) },
                { yLimiter.calculate(shapedAxis(-controller.leftX)) },
                { rotLimiter.calculate(shapedAxis(-controller.rightX)) },
                { false }
            )
        } else if (robotType == RobotType.SWERVE){
            val maxV = drivetrain.maxLinearSpeedMps
            val maxW = drivetrain.maxAngularSpeedRadPerSec
            drivetrain.defaultCommand =
                driveCommand(
                drivetrain,
                { xLimiter.calculate(shapedAxis(-controller.leftY)) * maxV},
                { yLimiter.calculate(shapedAxis(-controller.leftX)) * maxV },
                { rotLimiter.calculate(shapedAxis(-controller.rightX)) * maxW},
                { true }
            )
            (drivetrain as? SwerveDriveSubsystem)?.let { swerve ->
                controller.leftBumper().whileTrue(swerve.PARK_COMMAND)
            }
        }

        controller.leftTrigger().onTrue(launcher.INTAKE_COMMAND).onFalse(launcher.STOP_COMMAND)
        controller.rightTrigger().onTrue(launcher.LAUNCH_COMMAND).onFalse(launcher.STOP_COMMAND)
    }

    // -- Simulation --
    fun setupDashboard() {
        if (!isSim) return
        publishOnce("Field")
    }

    fun periodic() {
        val pose = drivetrain.getPose()
        setRobotPose(pose)
        publishGyroWidgets()

        if (isSim) {
            val truth = SimState.groundTruthPose
            val est = SimState.estimatedPose

            TelemetryRouter.pose(truth)
            TelemetryRouter.pose(est)
            TelemetryRouter.poseError(truth, est)
            TelemetryRouter.poseCompare(truth, est)
        }
    }

    private var lastYawDeg = 0.0
    private var lastYawTime = Timer.getFPGATimestamp()

    private fun publishGyroWidgets() {
        val yaw = drivetrain.getHeading()
        val now = Timer.getFPGATimestamp()
        val dt = now - lastYawTime

        val deltaDeg = yaw.minus(Rotation2d.fromDegrees(lastYawDeg)).degrees

        val yawRateDegPerSec =
            if (isSim) {
                SimSensors.measuredYawRateDegPerSec()
            } else {
                if (dt > 1e-6) deltaDeg / dt else 0.0
            }
        if (isSim) {
            TelemetryRouter.gyroTrue(
                SimSensors.trueYaw,
                SimSensors.measuredYaw(),
                SimSensors.trueYawRateDegPerSec
            )
            TelemetryRouter.gyro(yaw, yawRateDegPerSec)
        } else {
            TelemetryRouter.gyro(yaw, yawRateDegPerSec)
            SmartDashboard.putNumber("Gyro/PoseYawDeg", drivetrain.getPose().rotation.degrees)
        }


        lastYawDeg = yaw.degrees
        lastYawTime = now
    }


    fun simulationPeriodic() {
        val dt = dtSeconds()
        drivetrain.simulationPeriodic(dt)
    }
}