package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.HingeIO

class HingeSubsystem(private val io: HingeIO): SubsystemBase() {
    private val inputs = HingeIO.Inputs()

    private val limiter = HingeLimiter(
        0.0,
        180.0,
        1.0,
        10.0
    )

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setPercentLimited(requested: Double) {
        val safe = limiter.limit(inputs.angleDeg, requested = requested)
        io.setPercent(safe)
    }


    private class HingeLimiter(
        private val minAngle: Double,
        private val maxAngle: Double,
        private val softBufferDeg: Double = 1.0,
        private val slowZoneDeg: Double = 10.0
    ) {
        fun limit(angleDeg: Double, requested: Double): Double {
            val min = minAngle + softBufferDeg
            val max = maxAngle + softBufferDeg

            if (angleDeg >= max && requested > 0.0) return 0.0
            if (angleDeg <= min && requested < 0.0) return 0.0

            return when {
                requested > 0.0 && angleDeg > (max - slowZoneDeg) -> {
                    val remaining = (max - angleDeg).coerceIn(0.0, slowZoneDeg)
                    val scale = remaining / slowZoneDeg
                    requested * scale
                }
                requested < 0.0 && angleDeg < (min + slowZoneDeg) -> {
                    val remaining = (angleDeg - min).coerceIn(0.0, slowZoneDeg)
                    val scale = remaining / slowZoneDeg
                    requested * scale
                }
                else -> requested
            }
        }
    }
    fun manualUpCommand(): Command? =
        Commands.run({setPercentLimited(+0.4)}, this)

    fun manualDownCommmand(): Command? =
        Commands.run({setPercentLimited(-0.4)}, this)
}