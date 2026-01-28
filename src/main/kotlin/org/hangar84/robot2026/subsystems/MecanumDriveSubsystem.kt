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
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
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

    override fun getChassisSpeeds(): ChassisSpeeds = chassisSpeedsFromInputs

    private val TRACK_WIDTH = Inches.of(23.1875)

    private val WHEEL_BASE = Inches.of(16.125)

    private val WHEEL_BASE_M = WHEEL_BASE.`in`(Meters)

    private val TRACK_WIDTH_M = TRACK_WIDTH.`in`(Meters)

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

    private var frontLeftLocation: Translation2d = Translation2d(WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0)
    private var frontRightLocation: Translation2d = Translation2d(WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0)
    private var rearLeftLocation: Translation2d = Translation2d(-WHEEL_BASE_M / 2.0, TRACK_WIDTH_M / 2.0)
    private var rearRightLocation: Translation2d = Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0)

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

    private val DRIVE_FORWARD_COMMAND: Command =
        Commands.run(
            { drive(0.0, 0.3, 0.0, false) },
            this).withTimeout(2.5)

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

        // --- Chassis speeds ---
        val cs = chassisSpeedsFromInputs
        TelemetryRouter.chassisVel(
            cs.vxMetersPerSecond,
            cs.vyMetersPerSecond,
            cs.omegaRadiansPerSecond
        )

        // --- Swerve-Style Visualization & Power Data ---
        val table = NetworkTableInstance.getDefault().getTable("MecanumDrive")
        table.getEntry(".type").setString("SwerveDrive")

        val currentSpeeds = wheelSpeedsFromInputs()
        val measuredData = doubleArrayOf(
            0.0, currentSpeeds.frontLeftMetersPerSecond,
            0.0, currentSpeeds.frontRightMetersPerSecond,
            0.0, currentSpeeds.rearLeftMetersPerSecond,
            0.0, currentSpeeds.rearRightMetersPerSecond
        )
        table.getEntry("ModuleStates").setDoubleArray(measuredData)
        table.getEntry("RobotRotation").setDouble(getHeading().degrees)

        // Wheel Speeds
        val dataTable = table.getSubTable("ModuleData")
        dataTable.getEntry("FL_Speed").setDouble(mecanumInputs.flVelMps)
        dataTable.getEntry("FR_Speed").setDouble(mecanumInputs.frVelMps)
        dataTable.getEntry("RL_Speed").setDouble(mecanumInputs.rlVelMps)
        dataTable.getEntry("RR_Speed").setDouble(mecanumInputs.rrVelMps)

        // Power and Voltage
        val powerTable = table.getSubTable("Power")
        powerTable.getEntry("FL_Amps").setDouble(mecanumInputs.flCurrentAmps)
        powerTable.getEntry("FL_Volts").setDouble(mecanumInputs.flAppliedVolts)
        powerTable.getEntry("FR_Amps").setDouble(mecanumInputs.frCurrentAmps)
        powerTable.getEntry("FR_Volts").setDouble(mecanumInputs.frAppliedVolts)
        powerTable.getEntry("RL_Amps").setDouble(mecanumInputs.rlCurrentAmps)
        powerTable.getEntry("RL_Volts").setDouble(mecanumInputs.rlAppliedVolts)
        powerTable.getEntry("RR_Amps").setDouble(mecanumInputs.rrCurrentAmps)
        powerTable.getEntry("RR_Volts").setDouble(mecanumInputs.rrAppliedVolts)
    }

    override fun periodic() {
        TelemetryRouter.setBase(if (isSim) "${robotType.name}/Sim" else robotType.name)

        gyroIO.updateInputs(gyroInputs)
        mecanumIO.updateInputs(mecanumInputs)

        // Update odometry/pose estimator in periodic for real and sim
        val positions = wheelPositionsFromInputs()
        odometry.update(getHeading(), positions)
        poseEstimator.update(getHeading(), positions)

        publishMecanumTelemetry(positions)
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
            { poseEstimator.estimatedPosition },
            { pose -> resetPose(pose) },
            { chassisSpeedsFromInputs },
            this::driveRelative,
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            robotConfig,
            { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red },
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
    }

    override fun getPose(): Pose2d = poseEstimator.estimatedPosition
}