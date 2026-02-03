package org.hangar84.robot2026.subsystems

import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.LauncherIO
import org.hangar84.robot2026.telemetry.TelemetryRouter
import java.lang.reflect.Type

class LauncherSubsystem(val io: LauncherIO) : SubsystemBase() {

    private val inputs = LauncherIO.Inputs()
    private val isSim = RobotBase.isSimulation()

    private val mech = Mechanism2d(3.0, 3.0)
    private val root = mech.getRoot("launcher", 1.5, 1.5)

    private val visualWheel = root.append(
        MechanismLigament2d("Flywheel", 1.0, 0.0, 6.0, Color8Bit(0, 255, 0))
    )

    private val launcherTable = NetworkTableInstance.getDefault().getTable("Mechanism/Launcher")
    private val Launcher_Switch: GenericEntry = launcherTable.getTopic("Launcher Switch").getGenericEntry()
    private val Launcher_State: GenericEntry = launcherTable.getTopic("Launcher State").getGenericEntry()

    init {
        Launcher_State.setBoolean(false)
        Launcher_Switch.setBoolean(false)
        Trigger { Launcher_Switch.getBoolean(false) }
            .whileTrue(LAUNCH_COMMAND)
    }

    // - Commands -
    internal val LAUNCH_COMMAND
        get() = Commands.startEnd(
            { io.setPercent(1.0)
            Launcher_State.setBoolean(true)},
            { io.stop()
            Launcher_State.setBoolean(false)},
            this

        )

    fun pulseCommand(seconds: Double): Command =
        LAUNCH_COMMAND.withTimeout(seconds)
    override fun periodic() {
        io.updateInputs(inputs)

        if (isSim) {
            val fakeSpinSpeed = inputs.leftAppliedOutput * 20.0
            visualWheel.angle += fakeSpinSpeed

            SmartDashboard.putData("Mechanism 2D/Launcher Visualizer", mech)
        }

        TelemetryRouter.launcher(
            inputs.leftAppliedOutput,
            inputs.rightAppliedOutput,
            inputs.leftCurrentAmps,
            inputs.rightCurrentAmps,
            inputs.leftTempCelsius,
            inputs.rightTempCelsius
        )
    }
}