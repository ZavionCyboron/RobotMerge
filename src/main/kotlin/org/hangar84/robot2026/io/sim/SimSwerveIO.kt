package org.hangar84.robot2026.io.sim

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.hangar84.robot2026.io.SwerveIO
import kotlin.math.PI

class SimSwerveIO : SwerveIO {

    // internal simulated sensor state
    private val posMeters = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val velMps    = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    private val turnRad   = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val turnVelRadPerSec = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    // last commanded module states (from subsystem)
    private val desired = arrayOf(
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d())
    )

    override fun setModuleStates(fl: SwerveModuleState, fr: SwerveModuleState, rl: SwerveModuleState, rr: SwerveModuleState) {
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

    override fun simulationPeriodic(dtSeconds: Double) {
        val maxTurnRateRadPerSec = Math.toRadians(720.0) // same as your old 720 deg/sec

        for (i in 0..3) {
            // drive velocity directly follows command (simple model)
            val traction = 0.5
            velMps[i] = desired[i].speedMetersPerSecond * traction
            posMeters[i] += velMps[i] * dtSeconds

            // turn angle moves toward target at limited rate
            val target = desired[i].angle.radians
            val current = turnRad[i]
            val error = angleModulusRad(target - current)
            val delta = error.coerceIn(-maxTurnRateRadPerSec * dtSeconds, maxTurnRateRadPerSec * dtSeconds)

            val newAngle = current + delta
            turnVelRadPerSec[i] = delta / dtSeconds
            turnRad[i] = newAngle
        }
    }

    override fun updateInputs(inputs: SwerveIO.Inputs) {
        fun fill(m: SwerveIO.ModuleInputs, i: Int) {
            m.drivePosMeters = posMeters[i]
            m.driveVelMps = velMps[i]
            m.turnPosRad = turnRad[i]
            m.turnVelRadPerSec = turnVelRadPerSec[i]
        }

        fill(inputs.fl, 0)
        fill(inputs.fr, 1)
        fill(inputs.rl, 2)
        fill(inputs.rr, 3)
    }

    private fun angleModulusRad(radians: Double): Double {
        var x = radians
        while (x > PI) x -= 2.0 * PI
        while (x < -PI) x += 2.0 * PI
        return x
    }
}