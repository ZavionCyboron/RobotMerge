package org.hangar84.robot2026.io.sim.simdrivebaseio

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.hangar84.robot2026.io.interfaces.drivebaseio.SwerveIO
import kotlin.math.PI
import kotlin.math.abs

class SimSwerveIO : SwerveIO {

    // --- Drive Physics Constants ---
    private val nominalVoltage = 12.0
    private val maxSpeedMps = 4.8
    private val motorResistance = 0.02
    private val kv = nominalVoltage / maxSpeedMps
    private val stallCurrentLimit = 120.0

    private val drivePos = DoubleArray(4)
    private val driveVel = DoubleArray(4)
    private val turnPos = DoubleArray(4)
    private val turnVel = DoubleArray(4)

    private val driveAppliedVolts = DoubleArray(4)
    private val driveCurrentAmps = DoubleArray(4)
    private val turnAppliedVolts = DoubleArray(4)
    private val turnCurrentAmps = DoubleArray(4)

    private val desired = Array(4) {
        SwerveModuleState(0.0, Rotation2d())
    }

    override fun setModuleStates(
        fl: SwerveModuleState,
        fr: SwerveModuleState,
        rl: SwerveModuleState,
        rr: SwerveModuleState
    ) {
        desired[0] = fl
        desired[1] = fr
        desired[2] = rl
        desired[3] = rr
    }

    override fun stop() {
        setModuleStates(
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d())
        )
    }

    override fun simulationPeriodic(dt: Double) {
        if (dt <= 1e-6) return

        for (i in 0..3) {

            //Drive Voltage and Current
            val targetSpeed = desired[i].speedMetersPerSecond
            val percentOutput = (targetSpeed / maxSpeedMps)
                .coerceIn(-1.0, 1.0)

            val appliedVoltage = percentOutput * nominalVoltage
            driveAppliedVolts[i] = appliedVoltage

            val backEmf = kv * driveVel[i]
            val motorCurrent = ((appliedVoltage - backEmf) / motorResistance)
                .coerceIn(-stallCurrentLimit, stallCurrentLimit)

            driveCurrentAmps[i] = abs(motorCurrent)

            val accel = (appliedVoltage - backEmf) * 2.0
            driveVel[i] += accel * dt

            if (abs(percentOutput) < 0.02) {
                driveVel[i] *= 0.9
            }

            drivePos[i] += driveVel[i] * dt

            //Turn Voltage and Current
            val targetAngle = desired[i].angle.radians
            val error = angleModulus(targetAngle - turnPos[i])

            val turnKp = 8.0
            val turnVoltage = (turnKp * error)
                .coerceIn(-nominalVoltage, nominalVoltage)

            turnAppliedVolts[i] = turnVoltage

            val turnBackEmf = 2.0 * turnVel[i]
            val turnCurrent = ((turnVoltage - turnBackEmf) / 0.05)
                .coerceIn(-40.0, 40.0)

            turnCurrentAmps[i] = abs(turnCurrent)

            val turnAccel = (turnVoltage - turnBackEmf) * 4.0
            turnVel[i] += turnAccel * dt
            turnPos[i] += turnVel[i] * dt
        }
    }

    override fun updateInputs(inputs: SwerveIO.Inputs) {

        fun fill(m: SwerveIO.ModuleInputs, i: Int) {
            m.drivePosMeters = drivePos[i]
            m.driveVelMps = driveVel[i]
            m.turnPosRad = turnPos[i]

            m.driveAppliedVolts = driveAppliedVolts[i]
            m.driveCurrentAmps = driveCurrentAmps[i]

            m.turnAppliedVolts = turnAppliedVolts[i]
            m.turnCurrentAmps = turnCurrentAmps[i]
        }

        fill(inputs.fl, 0)
        fill(inputs.fr, 1)
        fill(inputs.rl, 2)
        fill(inputs.rr, 3)
    }

    private fun angleModulus(radians: Double): Double {
        var x = radians
        while (x > PI) x -= 2.0 * PI
        while (x < -PI) x += 2.0 * PI
        return x
    }
}
