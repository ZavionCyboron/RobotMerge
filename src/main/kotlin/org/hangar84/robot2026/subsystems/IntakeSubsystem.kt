package org.hangar84.robot2026.subsystems

import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.IntakeIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class IntakeSubsystem(val io: IntakeIO) : SubsystemBase() {

    private val inputs = IntakeIO.Inputs()
    private val isSim = RobotBase.isSimulation()

    private val mech = Mechanism2d(3.0, 3.0)
    private val root = mech.getRoot("Intake", 1.5, 1.5)

    private val visualWheel = root.append(
        MechanismLigament2d("Flywheel", 1.0, 0.0, 6.0, Color8Bit(0, 255, 0))
    )

    private val IntakeTable = NetworkTableInstance.getDefault().getTable("Mechanism/Intake")
    private val Intake_Switch: GenericEntry = IntakeTable.getTopic("Intake Switch").getGenericEntry()
    private val Intake_State: GenericEntry = IntakeTable.getTopic("Intake State").getGenericEntry()

    init {
        Intake_State.setBoolean(false)
        Intake_Switch.setBoolean(false)
        Trigger { Intake_Switch.getBoolean(false) }
            .whileTrue(INTAKE_COMMAND)
    }

    // - Commands -
    internal val INTAKE_COMMAND
        get() = Commands.startEnd(
            {
                io.setPercent(-1.0)
                Intake_State.setBoolean(true)
            },
            {
                io.stop()
                Intake_State.setBoolean(false)
            },
            this
        )

    override fun periodic() {
        io.updateInputs(inputs)

        if (isSim) {
            val fakeSpinSpeed = inputs.leftAppliedOutput * 20.0
            visualWheel.angle += fakeSpinSpeed

            SmartDashboard.putData("Mechanism 2D/Intake Visualizer", mech)
        }

        TelemetryRouter.Intake(
            inputs.leftAppliedOutput,
            inputs.leftCurrentAmps,
            inputs.leftTempCelcius
        )
    }
}