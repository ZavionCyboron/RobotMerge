package org.hangar84.robot2026.io.sim

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.hangar84.robot2026.io.IntakeIO
import org.hangar84.robot2026.io.LauncherIO

class SimIntakeIO : IntakeIO {
    private val launcherGearbox = DCMotor.getCIM(1)
    private val gearing = 1.0
    private val momentOfInertia = 0.0008

    private val currentLimitAmps = 40.0

    private val leftSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(launcherGearbox, momentOfInertia, gearing),
        launcherGearbox
    )

    private var appliedVoltsLeft = 0.0

    override fun setPercent(percent: Double) {
        val targetVolts = percent * 12.0

        appliedVoltsLeft = limitVoltage(leftSim, targetVolts)

        leftSim.setInputVoltage(appliedVoltsLeft)
    }

    private fun limitVoltage(sim: FlywheelSim, requestedVolts: Double): Double {
        val resistance = launcherGearbox.rOhms
        val kv = launcherGearbox.KvRadPerSecPerVolt

        val backEmfVolts = sim.angularVelocityRadPerSec / kv

        // Max voltage = (Limit * Resistance)
        val maxVolts = (currentLimitAmps * resistance) + backEmfVolts
        val minVolts = (-currentLimitAmps * resistance) + backEmfVolts

        return MathUtil.clamp(requestedVolts, minVolts, maxVolts)
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.leftAppliedOutput = appliedVoltsLeft / 12.0

        inputs.leftCurrentAmps = leftSim.currentDrawAmps

        inputs.leftTempCelcius = 22.0
    }

    fun simulationPeriodic(dtSeconds: Double) {
        leftSim.update(dtSeconds)
    }
}