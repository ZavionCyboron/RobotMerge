package org.hangar84.robot2026.io.real.drivebaserealio

import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Degrees
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.constants.Swerve
import org.hangar84.robot2026.io.interfaces.drivebaseio.SwerveIO
import org.hangar84.robot2026.subsystems.drivebases.swerve.`swerve-configs`.MAXSwerveModule
import org.hangar84.robot2026.subsystems.drivebases.swerve.`swerve-configs`.SwerveConfigs.drivingConfig
import org.hangar84.robot2026.subsystems.drivebases.swerve.`swerve-configs`.SwerveConfigs.turningConfig

class MaxSwerveIO(cfg: Swerve, maxcfg: MaxConfig) : SwerveIO {

    private val currentLimit = maxcfg.currentLimit + 10
    private val invertedTrue = maxcfg.inverted

    private val rrDrivingConfig = SparkMaxConfig().apply {
        apply(drivingConfig)
        inverted(invertedTrue) // true
        smartCurrentLimit(currentLimit) // 40 amps
    }

    private val fl: MAXSwerveModule = MAXSwerveModule(
        cfg.frontLeftDrivingId,
        cfg.frontLeftTurningId,
        Degrees.of(270.0),
        drivingConfig,
        turningConfig
    )

    private val fr: MAXSwerveModule = MAXSwerveModule(
        cfg.frontRightDrivingId,
        cfg.frontRightTurningId,
        Degrees.of(0.0),
        drivingConfig,
        turningConfig
    )

    private val rl: MAXSwerveModule = MAXSwerveModule(
        cfg.rearLeftDrivingId,
        cfg.rearLeftTurningId,
        Degrees.of(180.0),
        drivingConfig,
        turningConfig
    )

    private val rr: MAXSwerveModule = MAXSwerveModule(
        cfg.rearRightDrivingId,
        cfg.rearRightTurningId,
        Degrees.of(90.0),
        rrDrivingConfig,
        turningConfig
    )

    override fun updateInputs(inputs: SwerveIO.Inputs) {
        fun copy(module: MAXSwerveModule, out: SwerveIO.ModuleInputs) {


            val drive = module.drivingController
            val turn = module.turningController

            val pos = module.position
            // Driving encoder velocity
            val vel = drive.encoder.velocity

            out.drivePosMeters = pos.distanceMeters
            out.driveVelMps = vel
            out.turnPosRad = pos.angle.radians
            out.turnVelRadPerSec = 0.0

            out.driveTempC = drive.motorTemperature
            out.turnTempC = turn.motorTemperature

            val driveFaultBits = drive.faults.rawBits     // or drive.getFaults()
            val driveStickyBits = drive.stickyFaults.rawBits // or drive.getStickyFaults()
            val turnFaultBits = turn.faults.rawBits
            val turnStickyBits = turn.stickyFaults.rawBits

            out.driveFaulted = (driveFaultBits != 0) || (driveStickyBits != 0)
            out.turnFaulted  = (turnFaultBits != 0) || (turnStickyBits != 0)

            // --- Real Hardware Telemetry ---
            out.driveAppliedVolts = drive.appliedOutput * drive.busVoltage
            out.driveCurrentAmps = drive.outputCurrent

        }

        copy(fl, inputs.fl)
        copy(fr, inputs.fr)
        copy(rl, inputs.rl)
        copy(rr, inputs.rr)
    }

    override fun setModuleStates(
        fl: SwerveModuleState,
        fr: SwerveModuleState,
        rl: SwerveModuleState,
        rr: SwerveModuleState
    ) {
        this.fl.desiredState = fl
        this.fr.desiredState = fr
        this.rl.desiredState = rl
        this.rr.desiredState = rr
    }

    override fun stop() {
        fl.stop()
        fr.stop()
        rl.stop()
        rr.stop()
    }
}