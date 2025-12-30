package org.hangar84.robot2026.io

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveIO {
    data class ModuleInputs(
        var drivePosMeters: Double = 0.0,
        var driveVelMps: Double = 0.0,
        var turnPosRad: Double = 0.0,
        var turnVelRadPerSec: Double = 0.0
    )

    data class Inputs(
        val fl: ModuleInputs = ModuleInputs(),
        val fr: ModuleInputs = ModuleInputs(),
        val rl: ModuleInputs = ModuleInputs(),
        val rr: ModuleInputs = ModuleInputs()
    )
    fun updateInputs(inputs: Inputs)
    fun setModuleStates(
        fl: SwerveModuleState,
        fr: SwerveModuleState,
        rl: SwerveModuleState,
        rr: SwerveModuleState
    )
    fun stop()

    fun simulationPeriodic(dtSeconds: Double) {}
}