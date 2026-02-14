package org.hangar84.robot2026.subsystems.drivebases.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot2026.RobotContainer
import org.hangar84.robot2026.io.interfaces.drivebaseio.GyroIO
import org.hangar84.robot2026.io.interfaces.drivebaseio.SwerveIO
import org.hangar84.robot2026.subsystems.drivebases.Drivetrain
import org.hangar84.robot2026.subsystems.leds.LedSubsystem
import org.hangar84.robot2026.telemetry.TelemetryRouter
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.jvm.optionals.getOrNull

class SwerveDriveSubsystem(
    private val swerveIO: SwerveIO,
    private val gyroIO: GyroIO,
    private val leds: LedSubsystem
): Drivetrain() {

    companion object {
        internal val MAX_SPEED = Units.MetersPerSecond.of(4.8)
        internal val MAX_ANGULAR_SPEED = Units.RotationsPerSecond.of(1.0)
    }

    private val isSim = RobotBase.isSimulation()
    private val gyroInputs = GyroIO.Inputs()
    private val swerveInputs = SwerveIO.Inputs()

    private val anyDriveModuleFaulted =
        swerveInputs.fl.driveFaulted ||
        swerveInputs.fr.driveFaulted ||
        swerveInputs.rl.driveFaulted ||
        swerveInputs.rr.driveFaulted

    private val anyTurnModuleFaulted =
                swerveInputs.fl.turnFaulted ||
                swerveInputs.fr.turnFaulted ||
                swerveInputs.rl.turnFaulted ||
                swerveInputs.rr.turnFaulted


    override val maxLinearSpeedMps: Double = MAX_SPEED.`in`(Units.MetersPerSecond)
    override val maxAngularSpeedRadPerSec: Double = MAX_ANGULAR_SPEED.`in`(Units.RadiansPerSecond)

    private val WHEEL_BASE_M = Units.Inches.of(21.375).`in`(Units.Meters)
    private val TRACK_WIDTH_M = Units.Inches.of(19.50).`in`(Units.Meters)

    // -- Kinematics & Odometry --
    val kinematics = SwerveDriveKinematics(
        Translation2d(WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0),
        Translation2d(WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0),
        Translation2d(-WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0),
        Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0)
    )

    private val zeroPositions = arrayOf(
        SwerveModulePosition(), SwerveModulePosition(),
        SwerveModulePosition(), SwerveModulePosition()
    )

    internal var odometry = SwerveDriveOdometry(kinematics, Rotation2d(), zeroPositions)
    internal var poseEstimator = SwerveDrivePoseEstimator(
        kinematics, Rotation2d(), zeroPositions, Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(1.0, 1.0, 1.0)
    )

    // -- PhotonVision --
    private val camera = PhotonCamera("FrontCamera")
    private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
    private val cameraOffset = Transform3d(
        Translation3d(Units.Inches.of(-8.0), Units.Inches.of(9.0), Units.Inches.of(12.0)),
        Rotation3d(0.0, 0.0, 0.0)
    )
    private val photonEstimator = PhotonPoseEstimator(fieldLayout, cameraOffset)

    private val estimatedRobotPose: EstimatedRobotPose?
        get() = camera.allUnreadResults
            .asSequence()
            .mapNotNull { photonEstimator.estimateLowestAmbiguityPose(it).getOrNull() }
            .firstOrNull()

    private var commandedSpeeds = ChassisSpeeds()

    // -- Static Commands --
    internal val PARK_COMMAND: Command = Commands.run({
        val states = arrayOf(
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        )
        swerveIO.setModuleStates(states[0], states[1], states[2], states[3])
    }, this)

    private val DRIVE_FORWARD_COMMAND: Command = Commands.run(
        { drive(0.0, 0.3, 0.0, false) }, this
    ).withTimeout(2.5)

    init {
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_MaxSwerve)
    }

    override fun getHeading(): Rotation2d = gyroInputs.yaw
    override fun zeroHeading() = gyroIO.zeroYaw()
    override fun getPose(): Pose2d = if (isSim) odometry.poseMeters else poseEstimator.estimatedPosition

    // --- Implementation of getChassisSpeeds ---
    override fun getChassisSpeeds(): ChassisSpeeds {
        return kinematics.toChassisSpeeds(*moduleStatesFromInputs())
    }

    override fun resetPose(pose: Pose2d) {
        val positions = modulePositionsFromInputs()
        odometry.resetPosition(getHeading(), positions, pose)
        poseEstimator.resetPosition(getHeading(), positions, pose)
    }

    private fun modulePositionsFromInputs(): Array<SwerveModulePosition> = arrayOf(
        SwerveModulePosition(swerveInputs.fl.drivePosMeters, Rotation2d(swerveInputs.fl.turnPosRad)),
        SwerveModulePosition(swerveInputs.fr.drivePosMeters, Rotation2d(swerveInputs.fr.turnPosRad)),
        SwerveModulePosition(swerveInputs.rl.drivePosMeters, Rotation2d(swerveInputs.rl.turnPosRad)),
        SwerveModulePosition(swerveInputs.rr.drivePosMeters, Rotation2d(swerveInputs.rr.turnPosRad))
    )

    private fun moduleStatesFromInputs(): Array<SwerveModuleState> = arrayOf(
        SwerveModuleState(swerveInputs.fl.driveVelMps, Rotation2d(swerveInputs.fl.turnPosRad)),
        SwerveModuleState(swerveInputs.fr.driveVelMps, Rotation2d(swerveInputs.fr.turnPosRad)),
        SwerveModuleState(swerveInputs.rl.driveVelMps, Rotation2d(swerveInputs.rl.turnPosRad)),
        SwerveModuleState(swerveInputs.rr.driveVelMps, Rotation2d(swerveInputs.rr.turnPosRad))
    )

    override fun periodic() {
        TelemetryRouter.setBase(if (isSim) "${RobotContainer.robotType.name}/Sim" else RobotContainer.robotType.name)

        gyroIO.updateInputs(gyroInputs)
        swerveIO.updateInputs(swerveInputs)

        val positions = modulePositionsFromInputs()
        odometry.update(getHeading(), positions)
        poseEstimator.update(getHeading(), positions)

        // Vision Updates (Real World Only)
        if (!isSim) {
            estimatedRobotPose?.let { estimate ->
                SmartDashboard.putBoolean("Swerve/Vision/HasEstimate", true)
                poseEstimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds)
            } ?: SmartDashboard.putBoolean("Swerve/Vision/HasEstimate", false)
        }

        publishSwerveTelemetry()

        leds.setFault(LedSubsystem.Fault.DRIVE_MOTOR_FAIL, anyDriveModuleFaulted)
        leds.setFault(LedSubsystem.Fault.TURNING_MOTOR_FAIL, anyTurnModuleFaulted)
    }

    private fun formatAngle(angleDegrees: Double): Double {
        var wrapped = angleDegrees % 360.0
        if (wrapped < 0) wrapped += 360.0
        return wrapped
    }

    private fun publishSwerveTelemetry() {
        val pose = getPose()
        val chassis = getChassisSpeeds()
        val currentStates = moduleStatesFromInputs()

        TelemetryRouter.bool(RobotContainer.robotType.name, true)
        TelemetryRouter.pose(pose)

        val table = NetworkTableInstance.getDefault().getTable("SwerveDrive")
        table.getEntry(".type").setString("SwerveDrive")

        val measuredData = DoubleArray(8)
        for (i in 0..3) {
            val cleanAngle = currentStates[i].angle.degrees
            measuredData[i * 2] = cleanAngle
            measuredData[i * 2 + 1] = currentStates[i].speedMetersPerSecond
        }
        TelemetryRouter.SwerveDrive.moduleStates(measuredData)

        val names = arrayOf("FL", "FR", "RL", "RR")
        for (i in 0..3) {
            val cleanAngle = formatAngle(currentStates[i].angle.degrees)
            TelemetryRouter.SwerveDrive.data(
                "${names[i]} Angle Deg",
                "${names[i]} Speed Mps",
                angle = cleanAngle,
                currentStates[i].speedMetersPerSecond
            )
        }

        for (i in 0..3) {
            TelemetryRouter.SwerveDrive.power(
                "${names[i]} Drive Amps",
                "${names[i]} Volts",
                i
            )
        }

        // 4. Standard Telemetry Router Helpers
        TelemetryRouter.chassisVel(chassis.vxMetersPerSecond, chassis.vyMetersPerSecond, chassis.omegaRadiansPerSecond)
        TelemetryRouter.wheelEncoders(
            swerveInputs.fl.drivePosMeters, swerveInputs.fr.drivePosMeters,
            swerveInputs.rl.drivePosMeters, swerveInputs.rr.drivePosMeters,
            swerveInputs.fl.driveVelMps, swerveInputs.fr.driveVelMps,
            swerveInputs.rl.driveVelMps, swerveInputs.rr.driveVelMps
        )
    }

    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        commandedSpeeds = if (fieldRelative) {
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
        } else {
            ChassisSpeeds(xSpeed, ySpeed, rot)
        }
        if (isSim) gyroIO.setSimOmegaRadPerSec(commandedSpeeds.omegaRadiansPerSecond)

        val states = kinematics.toSwerveModuleStates(commandedSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearSpeedMps)
        swerveIO.setModuleStates(states[0], states[1], states[2], states[3])
    }

    fun driveRelative(chassisSpeeds: ChassisSpeeds) {
        commandedSpeeds = chassisSpeeds
        val states = kinematics.toSwerveModuleStates(chassisSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearSpeedMps)
        swerveIO.setModuleStates(states[0], states[1], states[2], states[3])
    }

    override fun buildAutoChooser(): SendableChooser<Command> {
        val robotConfig = try { RobotConfig.fromGUISettings() } catch (_: Exception) {
            return SendableChooser<Command>().apply { setDefaultOption("Drive Forward", DRIVE_FORWARD_COMMAND) }
        }

        AutoBuilder.configure(
            { getPose() },
            { resetPose(it) },
            { getChassisSpeeds() },
            this::driveRelative,
            PPHolonomicDriveController(PIDConstants(3.5, 0.0, 0.0), PIDConstants(4.0, 0.0, 0.1)),
            robotConfig,
            { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red },
            this
        )
        return AutoBuilder.buildAutoChooser().apply {
            addOption("Drive_Forward", DRIVE_FORWARD_COMMAND)
        }
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        if (!isSim) return
        swerveIO.simulationPeriodic(dtSeconds)
        gyroIO.simulationPeriodic(dtSeconds)
    }
}