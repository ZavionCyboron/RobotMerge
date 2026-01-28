package org.hangar84.robot2026

import CtreTwoValvePnematicsIO
import com.pathplanner.lib.auto.NamedCommands
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
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.commands.driveCommand
import org.hangar84.robot2026.constants.Constants
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.io.PneumaticsIO
import org.hangar84.robot2026.io.SwerveIO
import org.hangar84.robot2026.io.real.*
import org.hangar84.robot2026.io.sim.SimGyroIO
import org.hangar84.robot2026.io.sim.SimMecanumIO
import org.hangar84.robot2026.io.sim.SimPneumaticsIO
import org.hangar84.robot2026.io.sim.SimSwerveIO
import org.hangar84.robot2026.sim.*
import org.hangar84.robot2026.sim.SimClock.dtSeconds
import org.hangar84.robot2026.sim.SimField.publishOnce
import org.hangar84.robot2026.sim.SimField.setRobotPose
import org.hangar84.robot2026.sim.SimState.isSim
import org.hangar84.robot2026.subsystems.*
import org.hangar84.robot2026.telemetry.TelemetryRouter
import kotlin.math.withSign


object RobotContainer {
    private const val DRIVEDEADBAND = 0.08
    private val controller: CommandXboxController = CommandXboxController(0)
    private val buttonA = DigitalInput(19)

    val launcher = LauncherSubsystem(
        RevLauncherIO()
    )

    val intake = IntakeSubsystem(
         RevIntakeIO()
    )

    val pneumaticsIO: PneumaticsIO =
        if (isSim) SimPneumaticsIO()
        else CtreTwoValvePnematicsIO(
            pcmCanId = Constants.Pneumatics.PCM_CAN_ID,
            aExtend = Constants.Pneumatics.A_EXTEND_CHANNEL,
            aRetract = Constants.Pneumatics.A_RETRACT_CHANNEL,
            bExtend = Constants.Pneumatics.B_EXTEND_CHANNEL,
            bRetract = Constants.Pneumatics.B_RETRACT_CHANNEL,
        )

    val pneumatics = PneumaticsSubsystem(pneumaticsIO)

    private fun registerPathplannerEvents() {
        NamedCommands.registerCommand("Intake",intake.INTAKE_COMMAND)
        NamedCommands.registerCommand(
            "OctupleLaunch",
            Commands.sequence(
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Laundh 1st ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 2nd ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 3rd ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 4th ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 5th ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 6th ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 7th ball.
                launcher.pulseCommand(.25),
                Commands.waitSeconds(.4), // Launch 8th ball.
                launcher.pulseCommand(.25),
                )
        )

        NamedCommands.registerCommand("Lift", pneumatics.extendBothCommand())

        NamedCommands.registerCommand("Retract", pneumatics.retractBothCommand())
    }

    private fun readBootSelector(): RobotType {
        return if (buttonA.get()) {
            RobotType.SWERVE
        } else {
            RobotType.MECANUM
        }
    }

    val robotType: RobotType =
        if (RobotBase.isSimulation()) {
            RobotType.MECANUM
        } else {
            readBootSelector()
        }

    // The robot's subsystems
    val drivetrain: Drivetrain = when (robotType) {
        RobotType.SWERVE -> {
            val gyro: GyroIO = if (isSim) SimGyroIO() else AdisGyroIO().apply {
                setYawAdjustmentDegrees(90.0)
            }
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

    private var autoChooser: SendableChooser<Command>? = null



    val autonomousCommand: Command
        get() = autoChooser?.selected ?: InstantCommand()


    init {
        TelemetryRouter.setBase(
            if (isSim) {
                "${robotType.name}/Sim"
            } else {
                robotType.name
            }
        )

        if (isSim) {
            SimField.leftBluePose()
        }

        registerPathplannerEvents()

        autoChooser = drivetrain.buildAutoChooser()

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

        controller.leftTrigger(0.1).whileTrue(intake.INTAKE_COMMAND)
        controller.rightTrigger(0.1).whileTrue(launcher.LAUNCH_COMMAND)

        controller.a().onTrue(pneumatics.toggleBothCommand())
        controller.x().onTrue(pneumatics.extendBothCommand())
        controller.b().onTrue(pneumatics.retractBothCommand())
    }

    // -- Simulation --
    fun setupDashboard() {
        if (!isSim) return
        publishOnce("Field")
    }

    fun periodic() {
        val pose = drivetrain.getPose() // Fused orientation and position
        setRobotPose(pose)
        publishGyroWidgets()

        // --- Robot Orientation & Linear Velocity ---
        val table = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
            .getTable("RobotState")

        // Robot Orientation (Degrees)
        table.getEntry("GlobalOrientation").setDouble(pose.rotation.degrees)

        // Linear Velocity (Meters per Second)
        val speeds = drivetrain.getChassisSpeeds() // Ensure your Drivetrain interface has this
        val totalLinearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)

        table.getEntry("LinearVelocityMps").setDouble(totalLinearSpeed)
        table.getEntry("RotationSpeedDegPerSec").setDouble(Math.toDegrees(speeds.omegaRadiansPerSecond))

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