package org.hangar84.robot2026.telemetry

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import org.hangar84.robot2026.io.interfaces.drivebaseio.SwerveIO

object TelemetryRouter {
    private val isSim = RobotBase.isSimulation()
    private val lastPublishTime = mutableMapOf<String, Double>()

    private var base = ""


    private val nt = NetworkTableInstance.getDefault()

    private val launcherTable: NetworkTable =
        nt.getTable("Mechanism/Launcher")
    private val pneumaticsTable: NetworkTable = nt.getTable("Mechanism/Pneumatics")

    private val intakeTable: NetworkTable = nt.getTable("Mechanism/Intake")

    private val swerveTable = NetworkTableInstance.getDefault().getTable("SwerveDrive")

    private val powerTable = swerveTable.getSubTable("Power")

    object SwerveDrive {
        fun power(
            currentsName: String,
            voltsName: String,
            i: Int,
        ){
            val inputs = arrayOf(SwerveIO.Inputs().fl, SwerveIO.Inputs().fr, SwerveIO.Inputs().rl, SwerveIO.Inputs().rr)
            powerTable.getEntry(currentsName).setDouble(inputs[i].driveCurrentAmps)
            powerTable.getEntry(voltsName).setDouble(inputs[i].driveAppliedVolts)
        }
        fun data(
            angleName: String,
            speedName: String,
            angle: Double,
            speedMPS: Double
        ){
            val dataTable = swerveTable.getSubTable("ModuleData")
            dataTable.getEntry(angleName).setDouble(angle)
            dataTable.getEntry(speedName).setDouble(speedMPS)
        }

        fun moduleStates(
            measuredData: DoubleArray
        ){
            swerveTable.getEntry("ModuleStates").setDoubleArray(measuredData)
        }
    }

    private fun shouldPublish(group: String): Boolean {
        if (!TelemetryConfig.enabled(group, true)) return false

        val hz = TelemetryConfig.rateHz(group) ?: return true
        if (hz <= 0) return false

        val now = Timer.getFPGATimestamp()
        val minPeriod = 1.0 / hz
        val last = lastPublishTime[group] ?: Double.NEGATIVE_INFINITY

        if ((now - last) >= minPeriod) {
            lastPublishTime[group] = now
            return true
        }
        return false
    }

    fun setBase(robotType: String) {
        base = robotType
    }

    private fun publish(key: String, value: Double) {
        TelemetrySinks.publishNumber(key, value)
    }

    fun pose(pose: Pose2d) {
        if (!shouldPublish("pose")) return
        val prefix = TelemetryConfig.prefix("pose", "Pose")

        publish("$base/Pose/X", pose.x)
        publish("$base/Pose/Y", pose.y)
        publish("$base/Pose/Rot", pose.rotation.degrees)

        if (isSim) {
            SimTelemetry.pose(prefix, pose)
        } else {
            Telemetry.pose(prefix, pose)
        }
    }

    fun poseError(truth: Pose2d, est: Pose2d) {
        if (isSim) {
            if (!shouldPublish("pose")) return

            publish("$base/Pose/dX", truth.x)
            publish("$base/Pose/dY", est.y)
            publish("$base/Pose/dRot", est.rotation.minus(truth.rotation).degrees)

            SimTelemetry.poseError(
                TelemetryConfig.prefix("pose", "Pose"),
                truth, est
            )
        } else return
    }

    fun poseCompare(truth: Pose2d, est: Pose2d) {
        if (isSim) {
            if (!shouldPublish("pose")) return

            publish("$base/Pose/Truth", truth.x)
            publish("$base/Pose/Estimated", est.y)
            publish("$base/Pose/ErrorXY", truth.translation.getDistance(est.translation))
            publish("$base/Pose/ErrorDeg", truth.rotation.minus(est.rotation).degrees)

            SimTelemetry.poseCompare(
                TelemetryConfig.prefix("pose", "Pose"),
                truth, est
            )
        } else return
    }

    fun gyro(yaw: Rotation2d, yawRateDegPerSec: Double) {
        if (!shouldPublish("gyro")) return
        val prefix = TelemetryConfig.prefix("gyro", "Gyro")

        publish("$base/Gyro/YawDeg", yaw.degrees)
        publish("$base/Gyro/YawRateDegPerSec", yawRateDegPerSec)

        if (isSim) {
            SimTelemetry.gyro(prefix, yaw, yawRateDegPerSec)
        } else {
            Telemetry.gyro(prefix, yaw, yawRateDegPerSec)
        }
    }

    fun gyroTrue(trueYaw: Rotation2d, yaw: Rotation2d, trueYawRateDegPerSec: Double) {
        if (isSim) {
            if (!shouldPublish("gyro")) return

            publish("$base/TrueGyro/TrueYawDeg", trueYaw.degrees)
            publish("$base/TrueGyro/MeasuredYawDeg", yaw.degrees)
            publish("$base/TrueGyro/TrueYawRateDegPerSec", trueYawRateDegPerSec)

            SimTelemetry.gyroTrue(
                TelemetryConfig.prefix("gyro", "Gyro"),
                trueYaw, yaw, trueYawRateDegPerSec
            )
        } else return
    }

    fun chassisVel(vx: Double, vy: Double, omega: Double) {
        if (!shouldPublish("chassis")) return
        val prefix = TelemetryConfig.prefix("chassis", "Chassis")

        publish("$base/ChassisVel/VX", vx)
        publish("$base/ChassisVel/VY", vy)
        publish("$base/ChassisVel/Omega", omega)

        if (isSim) {
            SimTelemetry.chassisVel(prefix, vx, vy, omega)
        } else {
            Telemetry.chassisVel(prefix, vx, vy, omega)
        }
    }

    fun wheelEncoders(flPos: Double, frPos: Double, rlPos: Double, rrPos: Double,
               flVel: Double, frVel: Double, rlVel: Double, rrVel: Double
    ) {
        if (!shouldPublish("wheels")) return
        val prefix = TelemetryConfig.prefix("wheels", "Wheels")

        publish("$base/Pos/FL", flPos)
        publish("$base/Pos/FR", frPos)
        publish("$base/Pos/RL", rlPos)
        publish("$base/Pos/RR", rrPos)

        publish("$base/Vel/FL", flVel)
        publish("$base/Vel/FR", frVel)
        publish("$base/Vel/RL", rlVel)
        publish("$base/Vel/RR", rrVel)

        if (isSim) {
            SimTelemetry.wheelEncoders(
                prefix,
                flPos, frPos, rlPos, rrPos,
                flVel, frVel, rlVel, rrVel
            )
        } else {
            Telemetry.wheelEncoders(
                prefix,
                flPos, frPos, rlPos, rrPos,
                flVel, frVel, rlVel, rrVel
            )
        }
    }

    object Launcher{
        fun launcher(
            leftAppliedOutput: Double,
            rightAppliedOutput: Double,
            leftCurrentAmps: Double,
            rightCurrentAmps: Double,
            leftTempCelsius: Double,
            rightTempCelsius: Double,
            launcherState: Boolean,
            launcherSwitch: Boolean
        ) {
            if (!shouldPublish("launcher")) return

            val table = NetworkTableInstance.getDefault()
                .getTable("Mechanism/Launcher")

            table.getEntry("LeftAppliedVoltage").setDouble(leftAppliedOutput * 12.0)
            table.getEntry("RightAppliedVoltage").setDouble(rightAppliedOutput * 12.0)
            table.getEntry("LeftCurrentAmps").setDouble(leftCurrentAmps)
            table.getEntry("RightCurrentAmps").setDouble(rightCurrentAmps)
            table.getEntry("LeftTempCelsius").setDouble(leftTempCelsius)
            table.getEntry("RightTempCelsius").setDouble(rightTempCelsius)

            table.getEntry("Launcher State").setBoolean(launcherState)
            table.getEntry("Launcher Switch").setBoolean(launcherSwitch)
        }
        fun launcherSwitch(default: Boolean = false): Boolean =
            launcherTable.getEntry("Launcher Switch").getBoolean(default)
    }

    object Intake{
        val Intake_Switch: GenericEntry = intakeTable.getTopic("Intake Switch").getGenericEntry()
        val Intake_State: GenericEntry = intakeTable.getTopic("Intake State").getGenericEntry()
        fun intake(
            leftAppliedOutput: Double,
            leftCurrentAmps: Double,
            leftTempCelsius: Double,
        ) {
            if (!shouldPublish("Intake")) return

            val table = NetworkTableInstance.getDefault()
                .getTable("Mechanism/Intake")

            table.getEntry("LeftAppliedVoltage").setDouble(leftAppliedOutput * 12.0)
            table.getEntry("LeftCurrentAmps").setDouble(leftCurrentAmps)
            table.getEntry("LeftTempCelsius").setDouble(leftTempCelsius)
        }
    }

    object Pneumatics{

        val Extend_Left: GenericEntry =
            pneumaticsTable.getTopic("Extend Left").getGenericEntry()

        val Extend_Right: GenericEntry =
            pneumaticsTable.getTopic("Extend Right").getGenericEntry()

        val Extend_Both: GenericEntry =
            pneumaticsTable.getTopic("Extend Both").getGenericEntry()

        val setCompressor: GenericEntry =
            pneumaticsTable.getTopic("Toggle Compressor").getGenericEntry()
        fun pneumatics(
            CompressorEnabled: Boolean,
            Left_Solenoid_Extend: Boolean,
            Left_Solenoid_Retract: Boolean,
            Right_Solenoid_Extend: Boolean,
            Right_Solenoid_Retract: Boolean,
            Is_Left_Selected: Boolean,
            Is_Right_Selected: Boolean,
            Is_Both_Selected: Boolean,
            Selection: String,
            System_Enabled: Boolean,
        ) {
            if (!shouldPublish("Pneumatics")) return

            val Pneumatics_Table = NetworkTableInstance.getDefault()
                .getTable("Mechanism/Pneumatics")

            Pneumatics_Table.getEntry("Compressor Enabled").setBoolean(CompressorEnabled)
            Pneumatics_Table.getEntry("Left Solenoid Extend").setBoolean(Left_Solenoid_Extend)
            Pneumatics_Table.getEntry("Left Solenoid Retract").setBoolean(Left_Solenoid_Retract)
            Pneumatics_Table.getEntry("Right Solenoid Extend").setBoolean(Right_Solenoid_Extend)
            Pneumatics_Table.getEntry("Right Solenoid Retract").setBoolean(Right_Solenoid_Retract)

            Pneumatics_Table.getEntry("Selection/Is Left Selected").setBoolean(Is_Left_Selected)
            Pneumatics_Table.getEntry("Selection/Is Right Selected").setBoolean(Is_Right_Selected)
            Pneumatics_Table.getEntry("Selection/Is Both Selected").setBoolean(Is_Both_Selected)
            Pneumatics_Table.getEntry("Selection/Selection").setString(Selection)
            Pneumatics_Table.getEntry("Selection/System Enabled").setBoolean(System_Enabled)
        }
    }

    fun num(key: String, value: Double) {
        if (!shouldPublish("debug")) return

        if (isSim) {
            SimTelemetry.num(key, value)
        } else {
            Telemetry.num(key, value)
        }
    }

    fun bool(key: String, value: Boolean) {
        if (!shouldPublish("debug")) return

        if (isSim) {
            SimTelemetry.bool(key, value)
        } else {
            Telemetry.bool(key, value)
        }

    }
}