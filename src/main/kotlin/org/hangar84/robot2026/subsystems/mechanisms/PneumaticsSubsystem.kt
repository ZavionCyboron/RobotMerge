package org.hangar84.robot2026.subsystems.mechanisms

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.interfaces.mechanismio.PneumaticsIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class PneumaticsSubsystem(private val io: PneumaticsIO) : SubsystemBase() {

    private val inputs = PneumaticsIO.Inputs()
    enum class Selection { LEFT, RIGHT, BOTH }
    private var currentSelection = Selection.BOTH
    private val isLeftSelected get() =
        currentSelection == Selection.LEFT || currentSelection == Selection.BOTH

    private val isRightSelected get() =
        currentSelection == Selection.RIGHT || currentSelection == Selection.BOTH

    private val isBothSelected get() =
        currentSelection == Selection.BOTH

    private var systemEnabled = true
    fun isSystemEnabled(): Boolean = systemEnabled

    private val f = false

    init {
        TelemetryRouter.Pneumatics.Extend_Left.setBoolean(f)
        TelemetryRouter.Pneumatics.Extend_Right.setBoolean(f)
        TelemetryRouter.Pneumatics.Extend_Both.setBoolean(f)
        TelemetryRouter.Pneumatics.setCompressor.setBoolean(true)
        Trigger { TelemetryRouter.Pneumatics.Extend_Left.getBoolean(f) }
            .whileTrue(extendACommand().onlyIf { systemEnabled })
            .onFalse(retractACommand())

        Trigger{ TelemetryRouter.Pneumatics.Extend_Right.getBoolean(f) }
            .whileTrue(extendBCommand().onlyIf { systemEnabled })
            .onFalse(retractBCommand())

        Trigger{ TelemetryRouter.Pneumatics.Extend_Both.getBoolean(f) }
            .whileTrue(extendBothCommand().onlyIf { systemEnabled })
            .onFalse(retractBothCommand())

        Trigger{ TelemetryRouter.Pneumatics.setCompressor.getBoolean(true) }
            .whileTrue(enableCompressorCommand().onlyIf { systemEnabled })
            .onFalse(disableCompressorCommand())
    }

    override fun periodic() {
        io.updateInputs(inputs)

        TelemetryRouter.Pneumatics.pneumatics(
            inputs.CompressorEnabled, // Pass the table reference here
            inputs.Left_Solenoid_Extend,
            inputs.Left_Solenoid_Retract,
            inputs.Right_Solenoid_Extend,
            inputs.Right_Solenoid_Retract,
            isLeftSelected,
            isRightSelected,
            isBothSelected,
            currentSelection.name,
            systemEnabled
        )
    }

    fun setCompressor(enabled: Boolean) = io.setCompressor(enabled)

    fun enableCompressorCommand(): Command = Commands.runOnce({ setCompressor(true) }, this)
    fun disableCompressorCommand(): Command = Commands.runOnce({ setCompressor(f) }, this)

    fun setSystemEnabled(enabled: Boolean) {
        systemEnabled = enabled
        if (enabled)
            TelemetryRouter.Pneumatics.setCompressor.setBoolean(true)
        else
            TelemetryRouter.Pneumatics.setCompressor.setBoolean(f)
            TelemetryRouter.Pneumatics.Extend_Left.setBoolean(f)
            TelemetryRouter.Pneumatics.Extend_Right.setBoolean(f)
            TelemetryRouter.Pneumatics.Extend_Both.setBoolean(f)

    }

    fun setSelection(selection: Selection) {
        currentSelection = selection
    }

    fun cycleSelection() {
        val next = when (currentSelection) {
            Selection.LEFT -> Selection.RIGHT
            Selection.RIGHT -> Selection.BOTH
            Selection.BOTH -> Selection.LEFT
        }
        setSelection(next)
    }

    fun smartExtend() {
        if (!systemEnabled) return
        when(currentSelection) {
            Selection.LEFT -> extendA()
            Selection.RIGHT -> extendB()
            Selection.BOTH -> extendBoth()
        }
    }

    fun smartRetract() {
        if (!systemEnabled) return
        when(currentSelection) {
            Selection.LEFT -> retractA()
            Selection.RIGHT -> retractB()
            Selection.BOTH -> retractBoth()
        }
    }

    // ----- Actuator A -----
    fun extendA() = io.Left(PneumaticsIO.State.EXTEND)
    fun retractA() = io.Left(PneumaticsIO.State.RETRACT)
    fun neutralA() = io.Left(PneumaticsIO.State.NEUTRAL)

    fun toggleA() {
        val next =
            if (inputs.Left == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.Left(next)
    }

    // ----- Actuator B -----
    fun extendB() = io.Right(PneumaticsIO.State.EXTEND)
    fun retractB() = io.Right(PneumaticsIO.State.RETRACT)
    fun neutralB() = io.Right(PneumaticsIO.State.NEUTRAL)

    fun toggleB() {
        val next =
            if (inputs.Right == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.Right(next)
    }

    // ----- Both -----
    fun setBoth(state: PneumaticsIO.State) {
        io.Left(state)
        io.Right(state)
    }

    fun extendBoth() = setBoth(PneumaticsIO.State.EXTEND)
    fun retractBoth() = setBoth(PneumaticsIO.State.RETRACT)
    //fun neutralBoth() = setBoth(PneumaticsIO.State.NEUTRAL)

    fun toggleBoth() {
        val bothExtended =
            inputs.Left == PneumaticsIO.State.EXTEND &&
                    inputs.Right == PneumaticsIO.State.EXTEND

        setBoth(if (bothExtended) PneumaticsIO.State.RETRACT else PneumaticsIO.State.EXTEND)
    }

    // ----- Commands (nice for RobotContainer bindings) -----
    fun extendACommand(): Command = Commands.runOnce({ extendA() }, this)
    fun retractACommand(): Command = Commands.runOnce({ retractA() }, this)
    fun toggleACommand(): Command = Commands.runOnce({ toggleA() }, this)

    fun extendBCommand(): Command = Commands.runOnce({ extendB() }, this)
    fun retractBCommand(): Command = Commands.runOnce({ retractB() }, this)
    fun toggleBCommand(): Command = Commands.runOnce({ toggleB() }, this)

    fun extendBothCommand(): Command = Commands.runOnce({ extendBoth() }, this)
    fun retractBothCommand(): Command = Commands.runOnce({ retractBoth() }, this)
    fun toggleBothCommand(): Command = Commands.runOnce({ toggleBoth() }, this)
}