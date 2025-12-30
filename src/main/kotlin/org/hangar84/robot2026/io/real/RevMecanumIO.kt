package org.hangar84.robot2026.io.real

import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Constants.Mecanum
import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.mecanum.MecanumConfigs.driveConfig
import org.hangar84.robot2026.mecanum.MecanumModule

class RevMecanumIO : MecanumIO {

    private val rrConfig: SparkMaxConfig = SparkMaxConfig().apply() {
        apply(driveConfig)
        inverted(true)
    }
    private val fl = MecanumModule("FL", Mecanum.FRONT_LEFT_ID, driveConfig)
    private val fr = MecanumModule("FR", Mecanum.FRONT_RIGHT_ID, driveConfig)
    private val rl = MecanumModule("RL", Mecanum.REAR_LEFT_ID, driveConfig)
    private val rr = MecanumModule("RR", Mecanum.REAR_RIGHT_ID, rrConfig)

    override fun updateInputs(inputs: MecanumIO.Inputs) {
        inputs.flPosMeters = fl.positionMeters
        inputs.frPosMeters = fr.positionMeters
        inputs.rlPosMeters = rl.positionMeters
        inputs.rrPosMeters = rr.positionMeters

        inputs.flVelMps = fl.velocityMeters
        inputs.frVelMps = fr.velocityMeters
        inputs.rlVelMps = rl.velocityMeters
        inputs.rrVelMps = rr.velocityMeters
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