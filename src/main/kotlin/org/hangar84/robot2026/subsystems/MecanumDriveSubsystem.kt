package org.hangar84.robot2026.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot2026.RobotContainer.robotType
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.telemetry.TelemetryRouter
import kotlin.jvm.optionals.getOrNull
import org.hangar84.robot2026.io.GyroIO.Inputs as gyroInputs
import org.hangar84.robot2026.io.MecanumIO.Inputs as mecanumInputs


class MecanumDriveSubsystem(
    private val mecanumIO: MecanumIO,
    private val gyroIO: GyroIO
) :  Drivetrain() {

    // Aren't used but needed so that Drivetrain requirement is met
    override val maxAngularSpeedRadPerSec: Double = 2.0
    override val maxLinearSpeedMps: Double = 3.0

    private val isSim = RobotBase.isSimulation()

    private val gyroInputs = gyroInputs()
    private val mecanumInputs = mecanumInputs()

    override fun getHeading(): Rotation2d = gyroInputs.yaw

    override fun zeroHeading() {
        gyroIO.zeroYaw()
    }

    override fun resetPose(pose: Pose2d) {
        val positions = wheelPositionsFromInputs()
        odometry.resetPosition(getHeading(), positions, pose)
        poseEstimator.resetPosition(getHeading(), positions, pose)
    }

    private fun wheelPositionsFromInputs() = MecanumDriveWheelPositions(
        mecanumInputs.flPosMeters,
        mecanumInputs.frPosMeters,
        mecanumInputs.rlPosMeters,
        mecanumInputs.rrPosMeters
    )

    private fun wheelSpeedsFromInputs() = MecanumDriveWheelSpeeds(
        mecanumInputs.flVelMps,
        mecanumInputs.frVelMps,
        mecanumInputs.rlVelMps,
        mecanumInputs.rrVelMps
    )

    private var frontLeftLocation: Translation2d = Translation2d(0.833, 1.200)
    private var frontRightLocation: Translation2d = Translation2d(0.833, -1.200)
    private var rearLeftLocation: Translation2d = Translation2d(-0.833, 1.200)
    private var rearRightLocation: Translation2d = Translation2d(-0.833, -1.200)

    private val kinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        rearLeftLocation, rearRightLocation
    )

    private val chassisSpeedsFromInputs: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(wheelSpeedsFromInputs())

    private var odometry: MecanumDriveOdometry =
        MecanumDriveOdometry(
            kinematics,
            getHeading(),
            wheelPositionsFromInputs(),
            Pose2d()
        )

    private var poseEstimator: MecanumDrivePoseEstimator = MecanumDrivePoseEstimator(
        kinematics,
        getHeading(),
        wheelPositionsFromInputs(),
        Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations
        VecBuilder.fill(1.0, 1.0, 1.0), // Vision standard deviations
    )

    /*private val camera: PhotonCamera =  PhotonCamera("FrontCamera") 

    private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    private val cameraOffset =
        Transform3d(
            Translation3d(
                Inches.of(-8.0),
                Inches.of(9.0),
                Inches.of(12.0),
            ),
            Rotation3d(0.0, 0.0, 0.0),
        )*/

    private val DRIVE_FORWARD_COMMAND: Command =
        Commands.run(
            { drive(0.0, 0.3, 0.0, false) },
            this).withTimeout(2.5)
    init {

    }

    private fun publishMecanumTelemetry(wheelPositions: MecanumDriveWheelPositions) {
        TelemetryRouter.bool(robotType.name, true)

        // --- Heading / Pose ---
        val pose = poseEstimator.estimatedPosition

        TelemetryRouter.num("${robotType.name}/YawDeg", getHeading().degrees)
        TelemetryRouter.pose(pose)

        TelemetryRouter.wheelEncoders(wheelPositions.frontLeftMeters, wheelPositions.frontRightMeters,
            wheelPositions.rearLeftMeters, wheelPositions.rearRightMeters,
            mecanumInputs.flVelMps, mecanumInputs.frVelMps,
            mecanumInputs.rlVelMps, mecanumInputs.rrVelMps)

        // --- Chassis speeds (very useful to confirm math) ---
        val cs = chassisSpeedsFromInputs
        TelemetryRouter.chassisVel(
            cs.vxMetersPerSecond,
            cs.vyMetersPerSecond,
            cs.omegaRadiansPerSecond
        )
    }

    override fun periodic() {
        TelemetryRouter.setBase(if (isSim) "${robotType.name}/Sim" else robotType.name)

        gyroIO.updateInputs(gyroInputs)
        mecanumIO.updateInputs(mecanumInputs)

        // On real robot, update odometry here
        if (!isSim) {
            val positions = wheelPositionsFromInputs()
            odometry.update(getHeading(), positions)
            poseEstimator.update(getHeading(), positions)
        }

        // Telemetry works in both because it uses IO inputs
        publishMecanumTelemetry(wheelPositionsFromInputs())
    }

    private fun normalizeMecanum(w: MecanumDriveWheelSpeeds, max: Double): MecanumDriveWheelSpeeds {
        val maxMag = listOf(
            kotlin.math.abs(w.frontLeftMetersPerSecond),
            kotlin.math.abs(w.frontRightMetersPerSecond),
            kotlin.math.abs(w.rearLeftMetersPerSecond),
            kotlin.math.abs(w.rearRightMetersPerSecond)
        ).maxOrNull() ?: 0.0

        if (maxMag <= max || maxMag < 1e-9) return w

        val scale = max / maxMag
        return MecanumDriveWheelSpeeds(
            w.frontLeftMetersPerSecond * scale,
            w.frontRightMetersPerSecond * scale,
            w.rearLeftMetersPerSecond * scale,
            w.rearRightMetersPerSecond * scale
        )
    }

    fun driveRelative(relativeSpeeds: ChassisSpeeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(relativeSpeeds)
        wheelSpeeds = normalizeMecanum(wheelSpeeds, maxLinearSpeedMps)

        mecanumIO.setWheelSpeeds(
            wheelSpeeds.frontLeftMetersPerSecond,
            wheelSpeeds.frontRightMetersPerSecond,
            wheelSpeeds.rearLeftMetersPerSecond,
            wheelSpeeds.rearRightMetersPerSecond
        )
    }


    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        val speeds =
            if (fieldRelative) {
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
            } else {
                ChassisSpeeds(xSpeed, ySpeed, rot)
            }

        commandedSpeeds = speeds
        if (isSim) gyroIO.setSimOmegaRadPerSec(speeds.omegaRadiansPerSecond)

        var wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        wheelSpeeds = normalizeMecanum(wheelSpeeds, maxLinearSpeedMps)

        mecanumIO.setWheelSpeeds(
            wheelSpeeds.frontLeftMetersPerSecond,
            wheelSpeeds.frontRightMetersPerSecond,
            wheelSpeeds.rearLeftMetersPerSecond,
            wheelSpeeds.rearRightMetersPerSecond
        )
    }

    override fun buildAutoChooser(): SendableChooser<Command> {
        val robotConfig = try {
            RobotConfig.fromGUISettings()
        } catch (e: Exception) {
            DriverStation.reportError("PathPlanner RobotConfig missing/invalid: ${e.message}", e.stackTrace)
            return SendableChooser<Command>().apply {
                setDefaultOption("Drive Forward (Manual)", DRIVE_FORWARD_COMMAND)
            }
        }
        AutoBuilder.configure(
            // poseSupplier =
            { poseEstimator.estimatedPosition},
            // resetPose =
            { pose ->
                resetPose(pose)
            },
            // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
            // The following error should be ignored, since there is no way to remove/hide it.
            // robotRelativeSpeedsSupplier =
            { chassisSpeedsFromInputs },
            // output =
            this::driveRelative,
            // controller =
            PPHolonomicDriveController(
                // translationConstants =
                PIDConstants(5.0, 0.0, 0.0),
                // rotationConstants =
                PIDConstants(5.0, 0.0, 0.0),
            ),
            // robotConfig =
            robotConfig,
            // shouldFlipPath =
            { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red },
            // ...driveRequirements =
            this,
        )

        return AutoBuilder.buildAutoChooser().apply {
            addOption("Drive Forward (Manual)", DRIVE_FORWARD_COMMAND)
        }
    }
    private var commandedSpeeds = ChassisSpeeds()


    override fun simulationPeriodic(dtSeconds: Double) {
        if (!isSim) return

        mecanumIO.simulationPeriodic(dtSeconds)
        gyroIO.simulationPeriodic(dtSeconds)

        gyroIO.updateInputs(gyroInputs)
        mecanumIO.updateInputs(mecanumInputs)

        val positions = wheelPositionsFromInputs()
        odometry.update(getHeading(), positions)
        poseEstimator.update(getHeading(), positions)

        publishMecanumTelemetry(positions)
    }

    override fun getPose(): Pose2d = poseEstimator.estimatedPosition
}