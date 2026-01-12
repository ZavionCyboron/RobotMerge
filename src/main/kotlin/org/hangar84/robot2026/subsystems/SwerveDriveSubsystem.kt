package org.hangar84.robot2026.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot2026.RobotContainer.robotType
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.SwerveIO
import org.hangar84.robot2026.telemetry.TelemetryRouter
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.jvm.optionals.getOrNull
import org.hangar84.robot2026.io.GyroIO.Inputs as gyroInputs
import org.hangar84.robot2026.io.SwerveIO.Inputs as swerveInputs


class SwerveDriveSubsystem(
    private val swerveIO: SwerveIO,
    private val gyroIO: GyroIO

): Drivetrain() {
    // Constants
    companion object {
        internal val MAX_SPEED = MetersPerSecond.of(4.8)
        internal val MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.0)
    }

    private val isSim = RobotBase.isSimulation()

    private val gyroInputs = gyroInputs()
    private val swerveInputs = swerveInputs()

    override val maxLinearSpeedMps: Double = MAX_SPEED.`in`(MetersPerSecond)
    override val maxAngularSpeedRadPerSec: Double = MAX_ANGULAR_SPEED.`in`(RadiansPerSecond)

    private val WHEEL_BASE = Inches.of(24.0)
    private val TRACK_WIDTH = Inches.of(24.5)

    override fun getHeading(): Rotation2d = gyroInputs.yaw

    override fun zeroHeading() {
        gyroIO.zeroYaw()
    }

    override fun resetPose(pose: Pose2d) {
        val positions = modulePositionsFromInputs()
        odometry.resetPosition(getHeading(), positions, pose)
        poseEstimator.resetPosition(getHeading(), positions, pose)
    }

    private fun modulePositionsFromInputs(): Array<SwerveModulePosition> {
        fun pos(m: SwerveIO.ModuleInputs) =
            SwerveModulePosition(m.drivePosMeters, Rotation2d(m.turnPosRad))

        return arrayOf(
            pos(swerveInputs.fl),
            pos(swerveInputs.fr),
            pos(swerveInputs.rl),
            pos(swerveInputs.rr),
        )
    }

    private fun moduleStatesFromInputs(): Array<SwerveModuleState> {
        fun st(m: SwerveIO.ModuleInputs) =
            SwerveModuleState(m.driveVelMps, Rotation2d(m.turnPosRad))

        return arrayOf(
            st(swerveInputs.fl),
            st(swerveInputs.fr),
            st(swerveInputs.rl),
            st(swerveInputs.rr),
        )
    }


    // -- Odometry & Kinematics --

    val kinematics =
        SwerveDriveKinematics(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),  // FL
            Translation2d(-WHEEL_BASE / 2.0,  TRACK_WIDTH / 2.0),  // FR
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // RL
            Translation2d(-WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0), // RR
        )

    private val zeroPositions = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )

    internal var odometry: SwerveDriveOdometry =
        SwerveDriveOdometry(
            kinematics,
            getHeading(),
            zeroPositions
        )

    // -- PhotonVision --

    private val camera = PhotonCamera("FrontCamera")

    private val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    private val cameraOffset =
        Transform3d(
            Translation3d(
                Inches.of(-8.0),
                Inches.of(9.0),
                Inches.of(12.0),
            ),
            Rotation3d(0.0, 0.0, 0.0),
        )
    internal var poseEstimator: SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            zeroPositions,
            Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(1.0, 1.0, 1.0),
        )
    private val photonEstimator: PhotonPoseEstimator =
        PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraOffset
        )


    private val estimatedRobotPose: EstimatedRobotPose?
        get() {
            var estimate: EstimatedRobotPose? = null
            camera.allUnreadResults.forEach {
                estimate = photonEstimator.update(it).getOrNull()
            }

            return estimate
        }

    // - Static Commands -
    internal val PARK_COMMAND: Command = Commands.run({
        val states = arrayOf(
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),   // FL
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),  // FR
            SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),  // RL
            SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),   // RR
        )
        swerveIO.setModuleStates(states[0], states[1], states[2], states[3])
    }, this)

    private val DRIVE_FORWARD_COMMAND: Command =
        Commands.run(
            { drive(0.0, 0.3, 0.0, false) },
            this).withTimeout(2.5)


    init {
        HAL.report(
            tResourceType.kResourceType_RobotDrive,
            tInstances.kRobotDriveSwerve_MaxSwerve
        )
    }

    private fun publishSwerveTelemetry() {
        TelemetryRouter.bool(robotType.name, true)

        val pose = poseEstimator.estimatedPosition
        TelemetryRouter.num("${robotType.name}/YawDeg", getHeading().degrees)
        TelemetryRouter.pose(pose)

        val chassis = kinematics.toChassisSpeeds(*moduleStatesFromInputs())
        TelemetryRouter.chassisVel(
            chassis.vxMetersPerSecond,
            chassis.vyMetersPerSecond,
            chassis.omegaRadiansPerSecond
        )

        // Encoder + angle telemetry from IO
        TelemetryRouter.wheelEncoders(
            swerveInputs.fl.drivePosMeters, swerveInputs.fr.drivePosMeters,
            swerveInputs.rl.drivePosMeters, swerveInputs.rr.drivePosMeters,
            swerveInputs.fl.driveVelMps, swerveInputs.fr.driveVelMps,
            swerveInputs.rl.driveVelMps, swerveInputs.rr.driveVelMps
        )

        TelemetryRouter.angleDeg(
            Math.toDegrees(swerveInputs.fl.turnPosRad),
            Math.toDegrees(swerveInputs.fr.turnPosRad),
            Math.toDegrees(swerveInputs.rl.turnPosRad),
            Math.toDegrees(swerveInputs.rr.turnPosRad)
        )
    }

    override fun periodic() {
        TelemetryRouter.setBase(
            if (isSim) "${robotType.name}/Sim" else robotType.name
        )

        gyroIO.updateInputs(gyroInputs)
        swerveIO.updateInputs(swerveInputs)

        if (!isSim) {
            val positions = modulePositionsFromInputs()
            odometry.update(getHeading(), positions)
            poseEstimator.update(getHeading(), positions)

            val estimate = estimatedRobotPose
            SmartDashboard.putBoolean("Swerve/Vision/HasEstimate", estimate != null)
            if (estimate != null) {
                poseEstimator.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds
                )
            }

            publishSwerveTelemetry()
        }
    }

    // -- Commands --

        override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
            val speeds =
                if (fieldRelative) {
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
                } else {
                    ChassisSpeeds(xSpeed, ySpeed, rot)
                }

            commandedSpeeds = speeds
            if (isSim) gyroIO.setSimOmegaRadPerSec(speeds.omegaRadiansPerSecond)

            val states = kinematics.toSwerveModuleStates(speeds)
            SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearSpeedMps)

            swerveIO.setModuleStates(states[0], states[1], states[2], states[3])
        }

    fun driveRelative(chassisSpeeds: ChassisSpeeds) {
        commandedSpeeds = chassisSpeeds
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxLinearSpeedMps)
        swerveIO.setModuleStates(desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3])
    }

    override fun buildAutoChooser(): SendableChooser<Command> {
        if (isSim) return SendableChooser<Command>()

        val robotConfig = try {
            RobotConfig.fromGUISettings()
        } catch (e: Exception) {
            DriverStation.reportError("PathPlanner RobotConfig missing/invalid: ${e.message}", e.stackTrace)
            return SendableChooser<Command>().apply {
                setDefaultOption("Drive Forward (Manual)", DRIVE_FORWARD_COMMAND)
            }
        }
            AutoBuilder.configure(
                //poseSupplier =
                { poseEstimator.estimatedPosition },
                //resetPose =
                { pose -> resetPose(pose) },
                // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
                // The following error should be ignored, since there is no way to remove/hide it.
                //robotRelativeSpeedsSupplier =
                { kinematics.toChassisSpeeds(*moduleStatesFromInputs()) },
                //output =
                this::driveRelative,
                //controller =
                PPHolonomicDriveController(
                    // translationConstants =
                    PIDConstants(5.0, 0.0, 0.0),
                    // rotationConstants =
                    PIDConstants(5.0, 0.0, 0.0),
                ),
                //robotConfig =
                robotConfig,
                // shouldFlipPath =
                { DriverStation.getAlliance()?.getOrNull() == DriverStation.Alliance.Red },
                // ...driveRequirements =
                this,
            )

            return AutoBuilder.buildAutoChooser().apply {
                addOption("Drive Forward (Manual)", DRIVE_FORWARD_COMMAND)
            }
    }

    // -- Simulation --
    private var commandedSpeeds = ChassisSpeeds()


    override fun simulationPeriodic(dtSeconds: Double) {
        if (!isSim) return

        swerveIO.simulationPeriodic(dtSeconds)
        gyroIO.simulationPeriodic(dtSeconds)

        // Refresh inputs AFTER sim step
        gyroIO.updateInputs(gyroInputs)
        swerveIO.updateInputs(swerveInputs)

        // Now update odometry/estimator using the simulated sensors
        val positions = modulePositionsFromInputs()
        odometry.update(getHeading(), positions)
        poseEstimator.update(getHeading(), positions)

        publishSwerveTelemetry()
    }

    override fun getPose(): Pose2d = if (isSim) odometry.poseMeters else poseEstimator.estimatedPosition
}