package org.hangar84.robot2026.io.real

import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Degrees
import org.hangar84.robot2026.constants.Constants.Swerve
import org.hangar84.robot2026.io.SwerveIO
import org.hangar84.robot2026.swerve.MAXSwerveModule
import org.hangar84.robot2026.swerve.SwerveConfigs.drivingConfig
import org.hangar84.robot2026.swerve.SwerveConfigs.turningConfig

class MaxSwerveIO : SwerveIO {

    // SparkMax + encoder live ONLY here

    private val rrDrivingConfig = SparkMaxConfig().apply {
        apply(drivingConfig)
        inverted(true)
    }

    private val flDrivingConfig = SparkMaxConfig().apply {
        apply(drivingConfig)
        inverted(true)
    }

    private val fl: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_LEFT_DRIVING_ID,
        Swerve.FRONT_LEFT_TURNING_ID,
        Degrees.of(270.0),
        flDrivingConfig,
        turningConfig
    )


    private val fr: MAXSwerveModule = MAXSwerveModule(
        Swerve.FRONT_RIGHT_DRIVING_ID,
        Swerve.FRONT_RIGHT_TURNING_ID,
        Degrees.of(0.0),
        drivingConfig,
        turningConfig
    )

    private val rl: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_LEFT_DRIVING_ID,
        Swerve.REAR_LEFT_TURNING_ID,
        Degrees.of(180.0),
        drivingConfig,
        turningConfig
    )

    private val rr: MAXSwerveModule = MAXSwerveModule(
        Swerve.REAR_RIGHT_DRIVING_ID,
        Swerve.REAR_RIGHT_TURNING_ID,
        Degrees.of(90.0),
        rrDrivingConfig,
        turningConfig
    )

    override fun updateInputs(inputs: SwerveIO.Inputs) {
        fun copy(module: MAXSwerveModule, out: SwerveIO.ModuleInputs) {
            val pos = module.position
            val vel = module.drivingController.encoder.velocity

            out.drivePosMeters = pos.distanceMeters
            out.driveVelMps = vel
            out.turnPosRad = pos.angle.radians
            out.turnVelRadPerSec = 0.0
        }

        copy(fl, inputs.fl)
        copy(fr, inputs.fr)
        copy(rl, inputs.rl)
        copy(rr, inputs.rr)
    }

    override fun setModuleStates(
        fl: SwerveModuleState,
        fr: SwerveModuleState,
        rl: SwerveModuleState,
        rr: SwerveModuleState
    ) {
        this.fl.desiredState = fl
        this.fr.desiredState = fr
        this.rl.desiredState = rl
        this.rr.desiredState = rr
    }

    override fun stop() {
        fl.stop()
        fr.stop()
        rl.stop()
        rr.stop()
    }
}