package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.MechanisimIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class LauncherSubsystem(val io: MechanisimIO) : SubsystemBase() {

    private val inputs = MechanisimIO.Inputs()

    private val isSim = RobotBase.isSimulation()

    // - Commands -
    internal val LAUNCH_COMMAND
        get() = Commands.runOnce({ io.setPercent(1.0)}, this)

    internal val INTAKE_COMMAND
        get() = Commands.runOnce({ io.setPercent(-1.0)}, this)

    internal val STOP_COMMAND
        get() = Commands.runOnce({ io.stop() }, this)

    override fun periodic() {
        TelemetryRouter.setBase(
            if (isSim) {
                "Launcher/Sim"
            } else {
                "Launcher"
            }
        )

        io.updateInputs(inputs)

        // Optional telemetry (uses only inputs)
        TelemetryRouter.launcher(
            inputs.leftAppliedOutput, inputs.rightAppliedOutput,
            inputs.leftVelocityRpm, inputs.rightVelocityRpm,
            inputs.leftCurrentAmps, inputs.rightCurrentAmps
        )
    }

    override fun simulationPeriodic() {
        io.simulationPeriodic(0.02)
    }
}