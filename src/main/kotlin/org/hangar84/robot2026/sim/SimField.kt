package org.hangar84.robot2026.sim


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.RobotContainer.drivetrain
import org.hangar84.robot2026.RobotContainer.setupDashboard

object SimField {
    private val field = Field2d()
    private var published = false

    fun publishOnce(key: String = "Field") {
        if (published) return
        SmartDashboard.putData(key, field)
        published = true
    }

    fun setRobotPose(pose: Pose2d) {
        field.robotPose = pose
    }

    private fun applyStartPose(pose: Pose2d) {
        drivetrain.resetPose(pose)
        SimSensors.zeroGyro()
        SimSensors.setTrueYaw(pose.rotation, 0.0)  // order matters!
        setupDashboard()
        setRobotPose(pose)
        SimState.groundTruthPose = pose
        SimState.estimatedPose = pose
    }

    fun leftRedPose() {
        val leftRedPose = Pose2d(16.255476, 2.271337, Rotation2d.fromDegrees(180.0))
        applyStartPose(leftRedPose)
    }
    
    fun middleRedPose() {
        val middleRedPose = Pose2d(16.255476, 4.186562, Rotation2d.fromDegrees(180.0))
        applyStartPose(middleRedPose)
    }
    
    fun rightRedPose() {
        val rightRedPose = Pose2d(16.255476,5.742683, Rotation2d.fromDegrees(180.0))
        applyStartPose(rightRedPose)
    }
    
    fun rightBluePose() {
        val rightBluePose = Pose2d(1.113225, 2.271337, Rotation2d.fromDegrees(0.0))
        applyStartPose(rightBluePose)
    }
    
    fun middleBluePose() {
        val middleBluePose = Pose2d(1.113225, 4.186562, Rotation2d.fromDegrees(0.0))
        applyStartPose(middleBluePose)
    }
    
    fun leftBluePose() {
        val leftBluePose = Pose2d(1.113225, 5.742683, Rotation2d.fromDegrees(0.0))
        applyStartPose(leftBluePose)
    }
}