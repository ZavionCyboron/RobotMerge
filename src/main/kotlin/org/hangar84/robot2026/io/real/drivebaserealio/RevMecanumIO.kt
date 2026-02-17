package org.hangar84.robot2026.io.real.drivebaserealio

import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.io.interfaces.drivebaseio.MecanumIO
import org.hangar84.robot2026.subsystems.drivebases.mecanum.`mecanum-configs`.MecanumConfigs.driveConfig
import org.hangar84.robot2026.subsystems.drivebases.mecanum.`mecanum-configs`.MecanumModule
import org.hangar84.robot2026.constants.Mecanum

class RevMecanumIO(cfg: Mecanum, maxcfg: MaxConfig) : MecanumIO {
    private val invertedTrue = maxcfg.inverted

    private val rrConfig: SparkMaxConfig = SparkMaxConfig().apply() {
        apply(driveConfig)
        inverted(invertedTrue)
    }

    private val fl = MecanumModule("FL", cfg.frontLeftId, driveConfig)
    private val fr = MecanumModule("FR", cfg.frontRightId, driveConfig)
    private val rl = MecanumModule("RL", cfg.rearLeftId, driveConfig)
    private val rr = MecanumModule("RR", cfg.rearRightId, rrConfig)

    override fun updateInputs(inputs: MecanumIO.Inputs) {
        val flDrive = fl.motor
        val frDrive = fr.motor
        val rlDrive = rl.motor
        val rrDrive = rr.motor

        // Positions
        inputs.flPosMeters = fl.positionMeters
        inputs.frPosMeters = fr.positionMeters
        inputs.rlPosMeters = rl.positionMeters
        inputs.rrPosMeters = rr.positionMeters

        // Velocities
        inputs.flVelMps = fl.velocityMeters
        inputs.frVelMps = fr.velocityMeters
        inputs.rlVelMps = rl.velocityMeters
        inputs.rrVelMps = rr.velocityMeters

        // --- Voltage and Current ---

        // Front Left
        inputs.flAppliedVolts = flDrive.appliedOutput * fl.motor.busVoltage
        inputs.flCurrentAmps = flDrive.outputCurrent

        // Front Right
        inputs.frAppliedVolts = frDrive.appliedOutput * fr.motor.busVoltage
        inputs.frCurrentAmps = frDrive.outputCurrent

        // Rear Left
        inputs.rlAppliedVolts = rlDrive.appliedOutput * rl.motor.busVoltage
        inputs.rlCurrentAmps = rlDrive.outputCurrent

        // Rear Right
        inputs.rrAppliedVolts = rrDrive.appliedOutput * rr.motor.busVoltage
        inputs.rrCurrentAmps = rrDrive.outputCurrent

        inputs.flDriveTempC = flDrive.motorTemperature
        inputs.frDriveTempC = frDrive.motorTemperature
        inputs.rlDriveTempC = rlDrive.motorTemperature
        inputs.rrDriveTempC = rrDrive.motorTemperature

        val (flFaults, flStickyFaults) = flDrive.faults.rawBits to flDrive.stickyFaults.rawBits
        val (frFaults, frStickyFaults) = frDrive.faults.rawBits to frDrive.stickyFaults.rawBits
        val (rlFaults, rlStickyFaults) = rlDrive.faults.rawBits to rlDrive.stickyFaults.rawBits
        val (rrFaults, rrStickyFaults) = rrDrive.faults.rawBits to rrDrive.stickyFaults.rawBits

        inputs.flDriveFaulted = (flFaults != 0) || (flStickyFaults != 0)
        inputs.frDriveFaulted = (frFaults != 0) || (frStickyFaults != 0)
        inputs.rlDriveFaulted = (rlFaults != 0) || (rlStickyFaults != 0)
        inputs.rrDriveFaulted = (rrFaults != 0) || (rrStickyFaults != 0)
    }

    override fun setWheelSpeeds(
        fl: Double,
        fr: Double,
        rl: Double,
        rr: Double
    ) {
        this.fl.setVelocityMps(fl)
        this.fr.setVelocityMps(fr)
        this.rl.setVelocityMps(rl)
        this.rr.setVelocityMps(rr)
    }

    override fun stop() {
        fl.stop()
        fr.stop()
        rl.stop()
        rr.stop()
    }
}