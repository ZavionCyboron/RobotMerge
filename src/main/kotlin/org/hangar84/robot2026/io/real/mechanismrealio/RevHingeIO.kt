package org.hangar84.robot2026.io.real.mechanismrealio

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Hinge
import org.hangar84.robot2026.constants.MaxConfig
import org.hangar84.robot2026.io.interfaces.mechanismio.HingeIO

class RevHingeIO(cfg: Hinge, maxcfg: MaxConfig): HingeIO {

    private val currentLimit = maxcfg.currentLimit

    private val hinge_Motor = SparkMax(cfg.hingeMotorId, MotorType.kBrushed)

    private val gearRatio = 75

    init {
        val config = SparkMaxConfig().apply {
            smartCurrentLimit(currentLimit) // 30 amps
        }
        hinge_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        hinge_Motor.set(percent)
    }

    override fun updateInputs(inputs: HingeIO.Inputs) {
        val motorRotations = hinge_Motor.encoder.position

        val hingeRotations = motorRotations / gearRatio // the motors rotation divided by the gear ratio
        val hingeDegrees = hingeRotations * 360.0

        inputs.angleDeg = hingeDegrees
    }
}