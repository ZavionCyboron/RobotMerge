package org.hangar84.robot2026.io.sim.simmechanismio

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.hangar84.robot2026.io.interfaces.mechanismio.LauncherIO

class SimLauncherIO : LauncherIO {
    private val launcherGearbox = DCMotor.getCIM(1)
    private val gearing = 1.0
    private val momentOfInertia = 0.0008

    private val currentLimitAmps = 40.0

    private val leftSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(launcherGearbox, momentOfInertia, gearing),
        launcherGearbox
    )
    private val rightSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(launcherGearbox, momentOfInertia, gearing),
        launcherGearbox
    )

    private var appliedVoltsLeft = 0.0
    private var appliedVoltsRight = 0.0

    override fun setPercent(percent: Double) {
        val targetVolts = percent * 12.0

        appliedVoltsLeft = limitVoltage(leftSim, targetVolts)
        appliedVoltsRight = limitVoltage(rightSim, targetVolts)

        leftSim.setInputVoltage(appliedVoltsLeft)
        rightSim.setInputVoltage(appliedVoltsRight)
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

    override fun updateInputs(inputs: LauncherIO.Inputs) {
        inputs.leftAppliedOutput = appliedVoltsLeft / 12.0
        inputs.rightAppliedOutput = appliedVoltsRight / 12.0

        inputs.leftCurrentAmps = leftSim.currentDrawAmps
        inputs.rightCurrentAmps = rightSim.currentDrawAmps

        inputs.leftTempCelsius = 22.0
        inputs.rightTempCelsius = 22.0
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        leftSim.update(dtSeconds)
        rightSim.update(dtSeconds)
    }
}