package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.constants.Constants.Hinge
import org.hangar84.robot2026.io.HingeIO

class RevHingeIO: HingeIO {

    private val hinge_Motor = SparkMax(Hinge.Hinge_Motor, MotorType.kBrushed)

    private val gearRatio = 75

    init {
        val config = SparkMaxConfig().apply {
            smartCurrentLimit(30)
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