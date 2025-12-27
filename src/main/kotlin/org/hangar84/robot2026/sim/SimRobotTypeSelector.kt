package org.hangar84.robot2026.sim

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.constants.RobotType

object SimRobotTypeSelector {
    // Switch the RobotType."DriveBase" that you want to use to setDefaultOption and put the other one to addOption
    private val wantSwerve = true
    private val robotType = if (!wantSwerve) {
        RobotType.SWERVE
    } else {
        RobotType.MECANUM
    }
    private val chooser = SendableChooser<RobotType>().apply {
        setDefaultOption(robotType.name, robotType)
    }
    private var published = false

    // Publishes the sim robotType SmartDashboard once
    fun publishOnce() {
        if (!RobotBase.isSimulation() || published) return
        SmartDashboard.putData("Sim/RobotType", chooser)
        published = true
    }

    fun selected(): RobotType {
        publishOnce()
        return chooser.selected ?: RobotType.SWERVE
    }
}