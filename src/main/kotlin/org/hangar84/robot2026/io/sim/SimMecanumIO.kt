package org.hangar84.robot2026.io.sim

import org.hangar84.robot2026.io.MecanumIO
import org.hangar84.robot2026.sim.SimState

class SimMecanumIO : MecanumIO {

    private var cmdFL = 0.0
    private var cmdFR = 0.0
    private var cmdRL = 0.0
    private var cmdRR = 0.0

    override fun setWheelSpeeds(fl: Double, fr: Double, rl: Double, rr: Double) {
        cmdFL = fl; cmdFR = fr; cmdRL = rl; cmdRR = rr
    }

    override fun stop() = setWheelSpeeds(0.0, 0.0, 0.0, 0.0)

    private fun applyMotorLag(current: Double, target: Double, dt: Double): Double {
        val tau = 0.15
        return current + (target - current) * (dt / tau)
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        if (dtSeconds <= 1e-6) return

        // velocity follows command with lag
        SimState.simFLVel = applyMotorLag(SimState.simFLVel, cmdFL, dtSeconds)
        SimState.simFRVel = applyMotorLag(SimState.simFRVel, cmdFR, dtSeconds)
        SimState.simRLVel = applyMotorLag(SimState.simRLVel, cmdRL, dtSeconds)
        SimState.simRRVel = applyMotorLag(SimState.simRRVel, cmdRR, dtSeconds)

        // integrate wheel distance
        SimState.simFL += SimState.simFLVel * dtSeconds
        SimState.simFR += SimState.simFRVel * dtSeconds
        SimState.simRL += SimState.simRLVel * dtSeconds
        SimState.simRR += SimState.simRRVel * dtSeconds
    }

    override fun updateInputs(inputs: MecanumIO.Inputs) {
        inputs.flPosMeters = SimState.simFL
        inputs.frPosMeters = SimState.simFR
        inputs.rlPosMeters = SimState.simRL
        inputs.rrPosMeters = SimState.simRR

        inputs.flVelMps = SimState.simFLVel
        inputs.frVelMps = SimState.simFRVel
        inputs.rlVelMps = SimState.simRLVel
        inputs.rrVelMps = SimState.simRRVel
    }
}