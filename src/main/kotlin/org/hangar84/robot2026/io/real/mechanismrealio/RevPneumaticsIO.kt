package org.hangar84.robot2026.io.real.mechanismrealio
import edu.wpi.first.wpilibj.PneumaticHub
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.hangar84.robot2026.io.interfaces.mechanismio.PneumaticsIO
import org.hangar84.robot2026.constants.Pneumatics

class RevPneumaticsIO(cfg: Pneumatics) : PneumaticsIO {

    private val hub = PneumaticHub(cfg.revPHId)

    private val leftDouble = hub.makeDoubleSolenoid(cfg.aExtendChannel, cfg.aRetractChannel)
    private val rightDouble = hub.makeDoubleSolenoid(cfg.bExtendChannel, cfg.bRetractChannel)
    private var leftState = PneumaticsIO.State.NEUTRAL
    private var rightState = PneumaticsIO.State.NEUTRAL

    private fun PneumaticsIO.State.toWpi(): DoubleSolenoid.Value =
        when (this) {
            PneumaticsIO.State.EXTEND -> DoubleSolenoid.Value.kForward
            PneumaticsIO.State.RETRACT -> DoubleSolenoid.Value.kReverse
            PneumaticsIO.State.NEUTRAL -> DoubleSolenoid.Value.kOff
        }


    override fun Left(state: PneumaticsIO.State) {
        leftState = state
        leftDouble.set(state.toWpi())
    }

    override fun Right(state: PneumaticsIO.State) {
        rightState = state
        rightDouble.set(state.toWpi())
    }

    override fun setCompressor(enabled: Boolean) {
        if (enabled) {
            hub.enableCompressorDigital()
        } else {
            hub.disableCompressor()
        }
    }

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.Left = leftState
        inputs.Right = rightState

        inputs.CompressorEnabled = hub.compressor

        val aIsExtending = leftDouble.get()
        inputs.Left_Solenoid_Extend = aIsExtending == DoubleSolenoid.Value.kForward
        inputs.Left_Solenoid_Retract = aIsExtending == DoubleSolenoid.Value.kReverse

        val bIsExtending = rightDouble.get()
        inputs.Right_Solenoid_Extend = bIsExtending == DoubleSolenoid.Value.kForward
        inputs.Right_Solenoid_Retract = bIsExtending == DoubleSolenoid.Value.kReverse
    }
}