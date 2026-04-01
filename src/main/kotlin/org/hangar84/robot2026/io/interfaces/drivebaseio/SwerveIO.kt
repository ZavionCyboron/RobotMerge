package org.hangar84.robot2026.io.interfaces.drivebaseio

import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveIO {

    data class ModuleHealth(
        val driveFault: Boolean = false,
        val turnFault: Boolean = false,
        val driveStickyFault: Boolean = false,
        val turnStickyFault: Boolean = false,

        val driveConnected: Boolean = true,
        val turnConnected: Boolean = true,
        val absEncoderConnected: Boolean = true,

        val driveTempC: Double = 0.0,
        val turnTempC: Double = 0.0,
        val driveCurrentA: Double = 0.0,
        val turnCurrentA: Double = 0.0,
    )
    data class ModuleInputs(
        var drivePosMeters: Double = 0.0,
        var driveVelMps: Double = 0.0,
        var turnPosRad: Double = 0.0,
        var turnVelRadPerSec: Double = 0.0,

        var driveAppliedVolts: Double = 0.0,
        var driveCurrentAmps: Double = 0.0,

        var driveFaulted: Boolean = false,
        var turnFaulted: Boolean = false,
        var driveTempC: Double = 0.0,
        var turnTempC: Double = 0.0,

        var turnAppliedVolts: Double = 0.0,
        var turnCurrentAmps: Double = 0.0,

        val health: ModuleHealth = ModuleHealth()
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