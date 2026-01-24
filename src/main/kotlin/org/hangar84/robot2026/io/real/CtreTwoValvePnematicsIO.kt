import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import org.hangar84.robot2026.io.PneumaticsIO

class CtreTwoValvePnematicsIO(
    pcmCanId: Int,
    aExtend: Int, aRetract: Int,
    bExtend: Int, bRetract: Int,
) : PneumaticsIO {

    private val aExt = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, aExtend)
    private val aRet = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, aRetract)

    private val bExt = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, bExtend)
    private val bRet = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, bRetract)

    private var aState = PneumaticsIO.State.NEUTRAL
    private var bState = PneumaticsIO.State.NEUTRAL

    override fun setA(state: PneumaticsIO.State) {
        aState = state
        when (state) {
            PneumaticsIO.State.EXTEND -> { aRet.set(false); aExt.set(true) }
            PneumaticsIO.State.RETRACT -> { aExt.set(false); aRet.set(true) }
            PneumaticsIO.State.NEUTRAL -> { aExt.set(false); aRet.set(false) }
        }
    }

    override fun setB(state: PneumaticsIO.State) {
        bState = state
        when (state) {
            PneumaticsIO.State.EXTEND -> { bRet.set(false); bExt.set(true) }
            PneumaticsIO.State.RETRACT -> { bExt.set(false); bRet.set(true) }
            PneumaticsIO.State.NEUTRAL -> { bExt.set(false); bRet.set(false) }
        }
    }

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.aState = aState
        inputs.bState = bState
        inputs.aExtendOn = aExt.get()
        inputs.aRetractOn = aRet.get()
        inputs.bExtendOn = bExt.get()
        inputs.bRetractOn = bRet.get()
    }
}