package org.hangar84.robot2026.io.real

import com.revrobotics.ResetMode
import com.revrobotics.PersistMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import org.hangar84.robot2026.io.MechanisimIO

class RevMechanisimIO : MechanisimIO {

    private val leftController = SparkMax(9, MotorType.kBrushed)
    private val rightController = SparkMax(10, MotorType.kBrushed)

    init {
        val rightMotorConfig = SparkMaxConfig()
        rightMotorConfig.follow(leftController)
        rightMotorConfig.inverted(true)
        rightController.configure(
            rightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    override fun setPercent(percent: Double) {
        leftController.set(percent)
        // right follows in config
    }

    override fun updateInputs(inputs: MechanisimIO.Inputs) {
        inputs.leftAppliedOutput = leftController.appliedOutput
        inputs.rightAppliedOutput = rightController.appliedOutput

        // These are optional; if you don’t have encoders, leave 0.0
        // inputs.leftVelocityRpm = leftController.encoder.velocity
        // inputs.rightVelocityRpm = rightController.encoder.velocity

        inputs.leftCurrentAmps = leftController.outputCurrent
        inputs.rightCurrentAmps = rightController.outputCurrent
    }
}