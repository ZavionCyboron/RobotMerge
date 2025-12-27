package org.hangar84.robot2026.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.hangar84.robot2026.constants.Constants.Mecanum
import org.hangar84.robot2026.mecanum.MecanumConfigs.driveConfig
import org.hangar84.robot2026.mecanum.MecanumModule
import org.hangar84.robot2026.sim.SimSensors
import org.hangar84.robot2026.sim.SimState
import org.hangar84.robot2026.sim.SimState.estimatedPose
import org.hangar84.robot2026.sim.SimState.isSim
import org.hangar84.robot2026.sim.SimState.simFL
import org.hangar84.robot2026.sim.SimState.simFLVel
import org.hangar84.robot2026.sim.SimState.simFR
import org.hangar84.robot2026.sim.SimState.simFRVel
import org.hangar84.robot2026.sim.SimState.simRL
import org.hangar84.robot2026.sim.SimState.simRLVel
import org.hangar84.robot2026.sim.SimState.simRR
import org.hangar84.robot2026.sim.SimState.simRRVel
import org.hangar84.robot2026.telemetry.SimTelemetry
import org.hangar84.robot2026.telemetry.Telemetry
import kotlin.jvm.optionals.getOrNull
import org.hangar84.robot2026.sim.SimState.estimatedPose as simpose
import org.hangar84.robot2026.sim.SimState.yaw as simyaw

/*import org.photonvision.PhotonCamera
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.units.Units.Inches*/


class MecanumDriveSubsystem :  Drivetrain() {

    // Aren't used but needed so that Drivetrain requirement is met
    override val maxAngularSpeedRadPerSec: Double = 2.0
    override val maxLinearSpeedMps: Double = 3.0

    // Config update so that the rear right motor is inverted
    private val rearRightConfig: SparkMaxConfig = SparkMaxConfig().apply() {
        apply(driveConfig)
        inverted(true)
    }

    private val flMotor = MecanumModule("FrontLeft", Mecanum.FRONT_LEFT_ID, driveConfig)
    private val frMotor = MecanumModule("FrontRight", Mecanum.FRONT_RIGHT_ID, driveConfig)
    private val rlMotor = MecanumModule("RearLeft", Mecanum.REAR_LEFT_ID, driveConfig)
    private val rrMotor = MecanumModule("RearRight", Mecanum.REAR_RIGHT_ID, rearRightConfig)

    private val imu: ADIS16470_IMU? = if (isSim) null else ADIS16470_IMU()

    private val rotation2d
        get() = if (isSim) simyaw else Rotation2d.fromDegrees(imu!!.getAngle(imu.yawAxis))

    override fun resetPose(pose: Pose2d) {
        // Ground truth (sim)
        simpose = pose
        simyaw = pose.rotation

        // Reset estimators to match
        odometry.resetPosition(simyaw, simWheelPositions(), pose)
        poseEstimator.resetPosition(simyaw, simWheelPositions(), pose)

        // Optional: reset your wheel distances too if you treat simFL/simFR/... as distance
        simFL = 0.0; simFR = 0.0; simRL = 0.0; simRR = 0.0
        simFLVel = 0.0; simFRVel = 0.0; simRLVel = 0.0; simRRVel = 0.0
    }
    override fun zeroHeading() {
        imu?.reset()
    }

    override fun getHeading(): Rotation2d =
        if (isSim)
            SimSensors.measuredYaw()
        else
            rotation2d

    var drive: MecanumDrive =
        MecanumDrive(
            flMotor.motor, rlMotor.motor,
            frMotor.motor, rrMotor.motor
        )

    private var frontLeftLocation: Translation2d = Translation2d(0.833, 1.200)
    private var frontRightLocation: Translation2d = Translation2d(0.833, -1.200)
    private var rearLeftLocation: Translation2d = Translation2d(-0.833, 1.200)
    private var rearRightLocation: Translation2d = Translation2d(-0.833, -1.200)

    private val kinematics: MecanumDriveKinematics = MecanumDriveKinematics(
        frontLeftLocation, frontRightLocation,
        rearLeftLocation, rearRightLocation
    )

    private var odometry: MecanumDriveOdometry =
        MecanumDriveOdometry(
            kinematics,
            getHeading(),
            wheelPositions(),
            Pose2d()
        )

    private fun wheelPositions(): MecanumDriveWheelPositions =
        MecanumDriveWheelPositions(
            flMotor.positionMeters,
            frMotor.positionMeters,
            rlMotor.positionMeters,
            rrMotor.positionMeters
        )

    private var poseEstimator: MecanumDrivePoseEstimator = MecanumDrivePoseEstimator(
        kinematics,
        getHeading(),
        wheelPositions(),
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

    val chassisSpeeds: ChassisSpeeds
        get() =
            kinematics.toChassisSpeeds(
                MecanumDriveWheelSpeeds(
                    flMotor.velocityMeters, frMotor.velocityMeters,
                    rlMotor.velocityMeters, rrMotor.velocityMeters
                )
            )

    private val DRIVE_FORWARD_COMMAND: Command =
        Commands.run(
            { drive(0.0, 0.3, 0.0, false) },
            this).withTimeout(2.5)
    init {
        if (!isSim) {
            SmartDashboard.putData("IMU", imu)
        }

    }

    private fun publishMecanumTelemetry(wheelPositions: MecanumDriveWheelPositions) {
        // --- Heading / Pose ---
        val pose = poseEstimator.estimatedPosition

        Telemetry.num("Mecanum/YawDeg", getHeading().degrees)
        Telemetry.pose("Mecanum/Pose", pose)

        Telemetry.wheelEncoders(
            "Mecanum/Encoder",
            wheelPositions.frontLeftMeters, wheelPositions.frontRightMeters,
            wheelPositions.rearLeftMeters, wheelPositions.rearRightMeters,
            flMotor.velocityMeters, frMotor.velocityMeters,
            rlMotor.velocityMeters, rrMotor.velocityMeters
            )

        // --- Chassis speeds (very useful to confirm math) ---
        val cs = chassisSpeeds
        Telemetry.chassisVel(
            "Mecanum/Chassis",
            cs.vxMetersPerSecond,
            cs.vyMetersPerSecond,
            cs.omegaRadiansPerSecond
        )
    }

    private fun publishMecanumSimTelemetry(dtSeconds: Double) {
        SimTelemetry.bool("Mecanum/Sim", true)

        SimTelemetry.pose("Mecanum/Sim/Pose", simpose)
        SimTelemetry.num("Mecanum/Sim/YawDeg", getHeading().degrees)

        SimTelemetry.wheelVel(
            "Mecanum/Sim/Cmd",
            commandedSpeeds.vxMetersPerSecond,
            commandedSpeeds.vyMetersPerSecond,
            commandedSpeeds.omegaRadiansPerSecond
        )

        // Optional “motor outputs” (what you told the robot to do)
        SimTelemetry.speedMPS(
            "Mecanum/Sim/Motor/VoltsCmd",
            flMotor.velocityMeters,
            frMotor.velocityMeters,
            rlMotor.velocityMeters,
            rrMotor.velocityMeters
        )

        SimTelemetry.wheelEncoders(
            "Mecanum/Sim/Encoders",
            simFL, simFR, simRL, simRR,
            simFLVel, simFRVel, simRLVel, simRRVel
        )
        SimTelemetry.poseCompare(
            "Mecanum/Sim/PoseCompare",
            SimState.groundTruthPose,
            estimatedPose
        )

    }

    override fun periodic() {

        val wheelPositions = if (isSim) simWheelPositions() else wheelPositions()
        if (isSim) return

        odometry.update(
            getHeading(),
            wheelPositions
        )
        poseEstimator.update(getHeading(), wheelPositions)
        publishMecanumTelemetry(wheelPositions)
    }

    fun driveRelative(relativeSpeeds: ChassisSpeeds) {

        val wheelSpeeds = kinematics.toWheelSpeeds(relativeSpeeds)

        flMotor.setVelocityMps(wheelSpeeds.frontLeftMetersPerSecond)
        frMotor.setVelocityMps(wheelSpeeds.frontRightMetersPerSecond)
        rlMotor.setVelocityMps(wheelSpeeds.rearLeftMetersPerSecond)
        rrMotor.setVelocityMps(wheelSpeeds.rearRightMetersPerSecond)
    }


    override fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        // -- Simulation Mecanum Logic --
        commandedSpeeds = ChassisSpeeds(
            xSpeed,
            ySpeed,
            rot
        )

        if (isSim) return

        // -- Mecanum Logic --
        drive.driveCartesian(ySpeed, xSpeed, rot)
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
                val wheelPos = if (isSim) simWheelPositions() else wheelPositions()
                odometry.resetPosition(getHeading(), wheelPos, pose)
                poseEstimator.resetPose(pose)
            },
            // IntelliJ is off its rocker here. The spread operator works here, is practically required, and compiles.
            // The following error should be ignored, since there is no way to remove/hide it.
            // robotRelativeSpeedsSupplier =
            { chassisSpeeds },
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

    private fun simWheelPositions() =
        MecanumDriveWheelPositions(
            simFL,
            simFR,
            simRL,
            simRR
        )

    private fun applyMotorLag(current: Double, target: Double, dt: Double): Double {
        val tau = 0.15 // motor time constant
        return current + (target - current) * (dt / tau)
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        if (!isSim) return


        val dx = commandedSpeeds.vxMetersPerSecond * dtSeconds
        val dy = commandedSpeeds.vyMetersPerSecond * dtSeconds
        val dtheta = commandedSpeeds.omegaRadiansPerSecond * dtSeconds

        val newYawTruth = simyaw.plus(Rotation2d(dtheta))

        // yaw rate (deg/sec) for gyro sim
        val yawRateDegPerSecTruth = Math.toDegrees(commandedSpeeds.omegaRadiansPerSecond)

        // Update sim sensor drift/bias
        SimSensors.setTrueYaw(newYawTruth, yawRateDegPerSecTruth)
        SimSensors.update(dtSeconds)

        val ws = kinematics.toWheelSpeeds(commandedSpeeds)

        simFLVel = applyMotorLag(simFLVel, ws.frontLeftMetersPerSecond, dtSeconds)
        simFRVel = applyMotorLag(simFRVel, ws.frontRightMetersPerSecond, dtSeconds)
        simRLVel = applyMotorLag(simRLVel, ws.rearLeftMetersPerSecond, dtSeconds)
        simRRVel = applyMotorLag(simRRVel, ws.rearRightMetersPerSecond, dtSeconds)
        
        simFL += simFLVel * dtSeconds
        simFR += simFRVel * dtSeconds
        simRL += simRLVel * dtSeconds
        simRR += simRRVel * dtSeconds

        simyaw = newYawTruth
        val fieldDelta = Translation2d(dx, dy).rotateBy(simyaw.minus(Rotation2d(dtheta)))
        simpose = Pose2d(simpose.translation + fieldDelta, simyaw)

        // Build "measured" module positions + measured gyro
        val yawMeas = SimSensors.measuredYaw()

        val positionMeas = simWheelPositions()


        // Update odometry/poseEstimator with MEASURED sensors
        odometry.update(yawMeas, positionMeas)
        poseEstimator.update(yawMeas, positionMeas)

        // Store for comparison widgets
        SimState.groundTruthPose = simpose
        estimatedPose = poseEstimator.estimatedPosition

        publishMecanumSimTelemetry(dtSeconds)
    }

    override fun getPose(): Pose2d = poseEstimator.estimatedPosition
}