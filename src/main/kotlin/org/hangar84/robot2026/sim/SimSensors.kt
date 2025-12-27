package org.hangar84.robot2026.sim

import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI
import kotlin.math.cos
import kotlin.random.Random

object SimSensors {
    // --- Tunables ---
    //var encoderPosStdMeters = 0.002      // ~2mm noise
    //var encoderVelStdMps = 0.02          // 0.02 m/s noise
    var gyroYawStdDeg = 0.15              // ~0.6 deg noise
    var gyroRateStdDegPerSec = 2.0       // rate noise
    var gyroBiasDriftDegPerSec = 0.02    // slow bias drift (deg/sec)

    // deterministic randomness per run (change seed if you want)
    private val rng = Random(2026)

    // --- Ground truth (what physics says is happening) ---
    var trueYaw = Rotation2d()
        private set
    var trueYawRateDegPerSec = 0.0
        private set

    private var cachedMeasuredYaw = Rotation2d()
    private var cachedMeasuredYawRateDegPerSec = 0.0

    // --- What the robot "measures" after reset offsets + noise ---
    private var yawZeroOffset = Rotation2d()   // subtract this after "zero"
    private var gyroBiasDeg = 0.0              // drifting bias

    fun zeroGyro() {
        // Make current yaw read as 0 after this
        yawZeroOffset = trueYaw
    }

    fun setTrueYaw(yaw: Rotation2d, yawRateDegPerSec: Double) {
        trueYaw = yaw
        trueYawRateDegPerSec = yawRateDegPerSec
    }

    fun measuredYaw(): Rotation2d = cachedMeasuredYaw

    fun measuredYawRateDegPerSec(): Double = cachedMeasuredYawRateDegPerSec

    fun update(dtSeconds: Double) {
        // Bias drift (random walk)
        gyroBiasDeg += gyroBiasDriftDegPerSec * dtSeconds * gaussian(0.0, 1.0)

        cachedMeasuredYaw =
            Rotation2d.fromDegrees(
                (trueYaw.minus(yawZeroOffset)).degrees +
                        gaussian(0.0, gyroYawStdDeg) +
                        gyroBiasDeg
            )

        cachedMeasuredYawRateDegPerSec =
            trueYawRateDegPerSec + gaussian(0.0, gyroRateStdDegPerSec)
    }

    // --- Encoders ---
    data class EncoderTruth(var posMeters: Double = 0.0, var velMps: Double = 0.0)

    /*fun measuredPos(truth: EncoderTruth): Double =
        truth.posMeters + gaussian(0.0, encoderPosStdMeters)

    fun measuredVel(truth: EncoderTruth): Double =
        truth.velMps + gaussian(0.0, encoderVelStdMps)*/

    private fun gaussian(mean: Double, std: Double): Double {
        // Box–Muller
        val u1 = (rng.nextDouble().coerceAtLeast(1e-12))
        val u2 = rng.nextDouble()
        val z0 = kotlin.math.sqrt(-2.0 * kotlin.math.ln(u1)) * cos(2.0 * PI * u2)
        return mean + std * z0
    }
}