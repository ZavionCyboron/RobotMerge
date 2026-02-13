package org.hangar84.robot2026.io.real

import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.mecanum.MecanumConfigs.driveConfig
import org.hangar84.robot2026.mecanum.MecanumModule
import org.hangar84.robot2026.constants.Mecanum

class RevMecanumIO(cfg: Mecanum?, maxcfg: MaxConfig) : MecanumIO {
    private val invertedTrue = maxcfg.inverted

    private val rrConfig: SparkMaxConfig = SparkMaxConfig().apply() {
        apply(driveConfig)
        inverted(invertedTrue)
    }

    private val fl = MecanumModule("FL", cfg!!.frontLeftId, driveConfig)
    private val fr = MecanumModule("FR", cfg!!.frontRightId, driveConfig)
    private val rl = MecanumModule("RL", cfg!!.rearLeftId, driveConfig)
    private val rr = MecanumModule("RR", cfg!!.rearRightId, rrConfig)

    override fun updateInputs(inputs: MecanumIO.Inputs) {
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
        inputs.flAppliedVolts = fl.motor.appliedOutput * fl.motor.busVoltage
        inputs.flCurrentAmps = fl.motor.outputCurrent

        // Front Right
        inputs.frAppliedVolts = fr.motor.appliedOutput * fr.motor.busVoltage
        inputs.frCurrentAmps = fr.motor.outputCurrent

        // Rear Left
        inputs.rlAppliedVolts = rl.motor.appliedOutput * rl.motor.busVoltage
        inputs.rlCurrentAmps = rl.motor.outputCurrent

        // Rear Right
        inputs.rrAppliedVolts = rr.motor.appliedOutput * rr.motor.busVoltage
        inputs.rrCurrentAmps = rr.motor.outputCurrent
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