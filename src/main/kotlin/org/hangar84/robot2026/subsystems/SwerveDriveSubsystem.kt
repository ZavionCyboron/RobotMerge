package org.hangar84.robot2026.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.revrobotics.spark.config.SparkMaxConfig
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
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot2026.constants.Constants.Swerve
import org.hangar84.robot2026.swerve.MAXSwerveModule
import org.hangar84.robot2026.swerve.SwerveConfigs.drivingConfig
import org.hangar84.robot2026.swerve.SwerveConfigs.turningConfig
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.jvm.optionals.getOrNull


class SwerveDriveSubsystem :  Drivetrain() {
    // Constants
    companion object {
        internal val MAX_SPEED = MetersPerSecond.of(4.8)
        internal val MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.0)
    }

    private val WHEEL_BASE = Inches.of(24.0)
    private val TRACK_WIDTH = Inches.of(24.5)

    private val rearRightDrivingConfig = SparkMaxConfig().apply {
        apply(drivingConfig)
        inverted(true)
    }


    // Create MAXSwerveModules
    val frontLeft: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_LEFT_DRIVING_ID,
        Swerve.FRONT_LEFT_TURNING_ID,
        Degrees.of(270.0),
        drivingConfig,
        turningConfig
    )


    val frontRight: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_RIGHT_DRIVING_ID,
        Swerve.FRONT_RIGHT_TURNING_ID,
        Degrees.of(0.0),
        drivingConfig,
        turningConfig
    )

    val rearLeft: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_LEFT_DRIVING_ID,
        Swerve.REAR_LEFT_TURNING_ID,
        Degrees.of(180.0),
        drivingConfig,
        turningConfig
    )

    val rearRight: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_RIGHT_DRIVING_ID,
        Swerve.REAR_RIGHT_TURNING_ID,
        Degrees.of(90.0),
        rearRightDrivingConfig,
        turningConfig
    )


    // The gyro sensor

    private val allModules
        get() = arrayOf(frontLeft, frontRight, rearLeft, rearRight)
    private val allModulePositions: Array<SwerveModulePosition>
        get() = allModules.map { it.position }.toTypedArray()
    private val allModuleStates: Array<SwerveModuleState>
        get() = allModules.map { it.state }.toTypedArray()
    // -- Sensors --

    private val imu = ADIS16470_IMU()
    private val rotation
        get() = Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ))

    // -- Odometry & Kinematics --

    val kinematics =
        SwerveDriveKinematics(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        )

    internal var odometry: SwerveDriveOdometry =
        SwerveDriveOdometry(
            kinematics,
            rotation,
            allModulePositions
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
            rotation,
            allModulePositions,
            Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1), // State standard deviations
            VecBuilder.fill(1.0, 1.0, 1.0), // Vision standard deviations
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
    internal val PARK_COMMAND: Command = Commands.run ( {
        val positions =
            arrayOf(
                Rotation2d.fromDegrees(45.0), // Front left
                Rotation2d.fromDegrees(-45.0), // Front right
                Rotation2d.fromDegrees(-45.0), // Rear left
                Rotation2d.fromDegrees(45.0), // Rear right
            )

        allModules.forEachIndexed { i, module ->
            module.desiredState = SwerveModuleState(0.0, positions[i])
        }
    }, this )

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

    private fun publishModuleTelemetry(name: String, module: MAXSwerveModule) {
        val actual = module.state
        val desired = module.desiredState

        // --- Kinematics ---
        SmartDashboard.putNumber("$name/SpeedMps", actual.speedMetersPerSecond)
        SmartDashboard.putNumber("$name/AngleDeg", actual.angle.degrees)

        SmartDashboard.putNumber("$name/DesiredSpeedMps", desired.speedMetersPerSecond)
        SmartDashboard.putNumber("$name/DesiredAngleDeg", desired.angle.degrees)

        // --- Drive motor ---
        SmartDashboard.putNumber(
            "$name/DriveCurrent",
            module.drivingController.outputCurrent
        )
        SmartDashboard.putNumber(
            "$name/DriveOutput",
            module.drivingController.appliedOutput
        )
        SmartDashboard.putNumber(
            "$name/DriveTempC",
            module.drivingController.motorTemperature
        )

        // --- Turn motor ---
        SmartDashboard.putNumber(
            "$name/TurnCurrent",
            module.turningController.outputCurrent
        )
        SmartDashboard.putNumber(
            "$name/TurnOutput",
            module.turningController.appliedOutput
        )
        SmartDashboard.putNumber(
            "$name/TurnTempC",
            module.turningController.motorTemperature
        )
    }

    private fun publishSwerveTelemetry() {
        val pose = poseEstimator.estimatedPosition

        SmartDashboard.putNumber("Swerve/YawDeg", rotation.degrees)
        SmartDashboard.putNumber("Swerve/Pose/X", pose.x)
        SmartDashboard.putNumber("Swerve/Pose/Y", pose.y)
        SmartDashboard.putNumber("Swerve/Pose/HeadingDeg", pose.rotation.degrees)

        val chassis = kinematics.toChassisSpeeds(*allModuleStates)
        SmartDashboard.putNumber("Swerve/Chassis/Vx", chassis.vxMetersPerSecond)
        SmartDashboard.putNumber("Swerve/Chassis/Vy", chassis.vyMetersPerSecond)
        SmartDashboard.putNumber("Swerve/Chassis/Omega", chassis.omegaRadiansPerSecond)

        // Only compute estimate ONCE per periodic
        // (See periodic() change below)
        // SmartDashboard.putBoolean("Swerve/Vision/HasEstimate", ...)

        publishModuleTelemetry("Swerve/FL", frontLeft)
        publishModuleTelemetry("Swerve/FR", frontRight)
        publishModuleTelemetry("Swerve/RL", rearLeft)
        publishModuleTelemetry("Swerve/RR", rearRight)
    }

    override fun periodic() {


        odometry.update(rotation, allModulePositions)
        poseEstimator.update(rotation, allModulePositions)

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
    // -- Commands --

    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {

        val speedX = xSpeed
        val speedY = ySpeed
        val speedAngular = rot

        val swerveStates =
            kinematics.toSwerveModuleStates(
                if (fieldRelative) {
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speedX,
                        speedY,
                        speedAngular,
                        rotation
                    )
                } else {
                    ChassisSpeeds(speedX, speedY, speedAngular)
                },
            )

        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveStates,
            MAX_SPEED.`in`(MetersPerSecond)
        )

        allModules.forEachIndexed { i, module -> module.desiredState = swerveStates[i] }
    }

    fun driveRelative(chassisSpeeds: ChassisSpeeds) {
        val desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds)

        allModules.forEachIndexed { i, module -> module.desiredState = desiredStates[i] }
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
                //poseSupplier =
                { poseEstimator.estimatedPosition },
                //resetPose =
                { pose ->
                    odometry.resetPosition(rotation, allModulePositions, pose)
                    poseEstimator.resetPose(pose) },
                // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
                // The following error should be ignored, since there is no way to remove/hide it.
                //robotRelativeSpeedsSupplier =
                { kinematics.toChassisSpeeds(*allModuleStates) },
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
}
