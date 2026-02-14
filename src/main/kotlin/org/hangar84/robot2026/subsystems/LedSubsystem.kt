package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.LedIO
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.util.Color
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
        INTAKE_MOTOR_FAIL,
        LAUNCHER_MOTOR_FAIL,
        HINGE_MOTOR_FAIL
    }

    private var mode: Mode = Mode.DEFAULT
    private val faults: EnumSet<Fault> = EnumSet.noneOf(Fault::class.java)

    fun connect() = io.connect()

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
        when {
            faults.contains(Fault.RIO_BROWNOUT) -> {
                io.setStrobe(Color(1.0, 0.0, 0.0), 70) // red fast strobe
                return
            }
            faults.contains(Fault.DS_DISCONNECTED) -> {
                io.setChase(Color(0.7, 0.0, 1.0), 70) // purple chase
                return
            }
            faults.contains(Fault.LOW_BATTERY) -> {
                io.setBreathe(Color(1.0, 0.4, 0.0), 90) //orange breathe
            }
        }

        when (firstSubsystemFault()) {
            Fault.DRIVE_MOTOR_FAIL -> {io.setStrobe(Color(1.0, 1.0, 0.0), 120); return} // yellow strobe
            Fault.INTAKE_MOTOR_FAIL -> {io.setStrobe(Color(0.0, 1.0, 0.5), 120); return} // aquamarine strobe
            Fault.LAUNCHER_MOTOR_FAIL -> {io.setStrobe(Color(0.0, 0.7, 1.0), 120); return} // cyan strobe
            Fault.HINGE_MOTOR_FAIL -> {io.setStrobe(Color(1.0, 0.0, 1.0), 120); return} // magenta strobe
            null -> {}
            else -> {}
        }

        when (mode) {
            Mode.LAUNCH -> io.setStrobe(Color(1.0, 1.0, 1.0), 55)
            Mode.INTAKE -> io.setStrobe(Color(0.0, 1.0, 0.0), 35)
            Mode.DISABLED -> io.setBreathe(teamDim(), 90)
            Mode.DEFAULT -> io.setSolid(teamDim())
        }
    }

    private fun firstSubsystemFault(): Fault? {
        val order = listOf(
            Fault.DRIVE_MOTOR_FAIL,
            Fault.INTAKE_MOTOR_FAIL,
            Fault.LAUNCHER_MOTOR_FAIL,
            Fault.HINGE_MOTOR_FAIL
        )
        return order.firstOrNull { faults.contains(it)}
    }

    private fun teamDim(): Color {
        val a = DriverStation.getAlliance()
        if (a.isPresent) {
            return if (a.get() == DriverStation.Alliance.Red) Color(0.35, 0.0, 0.0)
            else Color(0.0, 0.0, 0.35)
        }
        return Color(0.15, 0.15, 0.15)
    }
}