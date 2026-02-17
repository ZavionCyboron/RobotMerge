package org.hangar84.robot2026.subsystems.mechanisms

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.interfaces.mechanismio.IntakeIO
import org.hangar84.robot2026.telemetry.TelemetryRouter
import org.hangar84.robot2026.telemetry.TelemetryRouter.Intake

class IntakeSubsystem(val io: IntakeIO) : SubsystemBase() {

    private val inputs = IntakeIO.Inputs()
    private val isSim = RobotBase.isSimulation()

    private val mech = Mechanism2d(3.0, 3.0)
    private val root = mech.getRoot("Intake", 1.5, 1.5)

    private val visualWheel = root.append(
        MechanismLigament2d("Flywheel", 1.0, 0.0, 6.0, Color8Bit(0, 255, 0))
    )

    init {
        Intake.Intake_State.setBoolean(false)
        Intake.Intake_Switch.setBoolean(false)
        Trigger { Intake.Intake_Switch.getBoolean(false) }
            .whileTrue(INTAKE_COMMAND)
    }

    // - Commands -
    internal val INTAKE_COMMAND
        get() = Commands.startEnd(
            {
                io.setPercent(-1.0)
                Intake.Intake_State.setBoolean(true)
            },
            {
                io.stop()
                Intake.Intake_State.setBoolean(false)
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

        Intake.intake(
            inputs.leftAppliedOutput,
            inputs.leftCurrentAmps,
            inputs.leftTempCelcius,
            TelemetryRouter.Intake.getIntakeSpeed()
        )
    }
}