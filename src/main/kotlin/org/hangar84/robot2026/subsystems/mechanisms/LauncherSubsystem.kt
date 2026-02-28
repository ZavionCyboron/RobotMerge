package org.hangar84.robot2026.subsystems.mechanisms

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.interfaces.mechanismio.LauncherIO
import org.hangar84.robot2026.telemetry.TelemetryRouter.Launcher

class LauncherSubsystem(val io: LauncherIO) : SubsystemBase() {

    private val inputs = LauncherIO.Inputs()
    private val isSim = RobotBase.isSimulation()

    private val mech = Mechanism2d(3.0, 3.0)
    private val root = mech.getRoot("launcher", 1.5, 1.5)
    private var launch_State = false
    private var launch_Switch = false

    private val visualWheel = root.append(
        MechanismLigament2d("Flywheel", 1.0, 0.0, 6.0, Color8Bit(0, 255, 0))
    )
    init {
        launch_State = false
        Trigger {  Launcher.launcherSwitch(launch_Switch) }
            .whileTrue(LAUNCH_COMMAND)
    }

    // - Commands -
    internal val LAUNCH_COMMAND
        get() = Commands.sequence(
            Commands.runOnce({
                launch_State = true
                io.setRightPercent(1.0)
            }, this),

            Commands.waitSeconds(.5),

            Commands.runOnce({
                io.setLeftPercent(1.0)
            }, this),

            Commands.run({}, this),
        ).finallyDo { _: Boolean ->
            io.stop()
            launch_State = false
        }

    fun pulseCommand(seconds: Double): Command =
        LAUNCH_COMMAND.withTimeout(seconds)
    override fun periodic() {
        io.updateInputs(inputs)

        if (isSim) {
            val fakeSpinSpeed = inputs.leftAppliedOutput * 20.0
            visualWheel.angle += fakeSpinSpeed

            SmartDashboard.putData("Mechanism 2D/Launcher Visualizer", mech)
        }

       Launcher.launcher(
            inputs.leftAppliedOutput,
            inputs.rightAppliedOutput,
            inputs.leftCurrentAmps,
            inputs.rightCurrentAmps,
            inputs.leftTempCelsius,
            inputs.rightTempCelsius,
            launch_State,
            Launcher.launcherSwitch(launch_Switch),
            Launcher.getLauncherSpeed()
        )
    }
}