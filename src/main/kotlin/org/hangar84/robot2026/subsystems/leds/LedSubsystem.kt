package org.hangar84.robot2026.subsystems.leds

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.interfaces.ledio.LedIO
import org.hangar84.robot2026.io.interfaces.ledio.LedTarget
import java.util.EnumSet

class LedSubsystem(private val io: LedIO): SubsystemBase() {
    enum class Mode { DISABLED, DEFAULT, INTAKE, LAUNCH }

    enum class Fault {
        // RoboRIO / system
        RIO_BROWNOUT,
        DS_DISCONNECTED,
        LOW_BATTERY,

        // Subsystem faults (add as many as you prefer)
        DRIVE_MOTOR_FAIL,
        TURNING_MOTOR_FAIL, //Swerve specific
        INTAKE_MOTOR_FAIL,
        LAUNCHER_MOTOR_FAIL,
        HINGE_MOTOR_FAIL
    }
    private var mode: Mode = Mode.DEFAULT
    private val faults: EnumSet<Fault> = EnumSet.noneOf(Fault::class.java)

    init {
        io.connect()
    }

    fun setMode(newMode: Mode) {mode = newMode}

    fun setFault(fault: Fault, active: Boolean) {
        if (active) faults.add(fault) else faults.remove(fault)
    }

    override fun periodic() {
        setFault(Fault.RIO_BROWNOUT, RobotController.isBrownedOut())
        setFault(Fault.DS_DISCONNECTED, !DriverStation.isDSAttached())
        setFault(Fault.LOW_BATTERY, RobotController.getBatteryVoltage() < 11.2)

        applyPriority()
    }

    private fun applyPriority() {
        // --- PRIORITY 1: MAIN ERRORS (Zia + Strips) ---
        when {
            faults.contains(Fault.RIO_BROWNOUT) -> {
                io.setStrobe(LedTarget.BASE, Color.kRed, 70)
                return
            }
            faults.contains(Fault.DS_DISCONNECTED) -> {
                io.setChase(LedTarget.BASE, Color.kPurple, 70)
                return
            }
            faults.contains(Fault.LOW_BATTERY) -> {
                io.setBreathe(LedTarget.BASE, Color.kOrange, 90)
                return
            }
        }

        // --- PRIORITY 2: SUBSYSTEM ERRORS (Strips Only) ---
        when {
            faults.contains(Fault.DRIVE_MOTOR_FAIL) -> {
                io.setStrobe(LedTarget.BASE, Color.kYellow, 120)
                return
            }
            faults.contains(Fault.INTAKE_MOTOR_FAIL) -> {
                io.setStrobe(LedTarget.INTAKE, Color.kAquamarine, 120)
                return
            }
            faults.contains(Fault.LAUNCHER_MOTOR_FAIL) -> {
                io.setStrobe(LedTarget.LAUNCHER, Color.kCyan, 120)
                return
            }
            faults.contains(Fault.HINGE_MOTOR_FAIL) -> {
                io.setStrobe(LedTarget.BASE, Color.kDarkOrange, 120)
            }
        }

        // --- PRIORITY 3: NORMAL MODES ---
        when (mode) {
            Mode.LAUNCH -> io.setStrobe(LedTarget.LAUNCHER, Color.kWhite, 55)
            Mode.INTAKE -> io.setStrobe(LedTarget.INTAKE, Color.kGreen, 35)
            Mode.DISABLED -> io.setBreathe(LedTarget.ALL, teamDim(), 90)
            Mode.DEFAULT -> io.setSolid(LedTarget.ALL, teamDim())
        }
    }

    private fun teamDim(): Color {
        val a = DriverStation.getAlliance()
        if (a.isPresent) {
            return if (a.get() == DriverStation.Alliance.Red) Color(0.35, 0.0, 0.0) // dark red
            else Color(0.0, 0.0, 0.35)
        }
        return Color(0.15, 0.15, 1.0) // Grayscale 15%
    }
}