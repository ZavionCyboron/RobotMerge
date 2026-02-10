package org.hangar84.robot2026.constants

object Constants {

    object Hinge {
        const val Hinge_Motor = 18
    }

    object Pneumatics {
        const val REVPH_CAN_ID = 17          // <-- your REVPH CanID
        const val A_EXTEND_CHANNEL = 0      // solenoid channel
        const val A_RETRACT_CHANNEL = 1
        const val B_EXTEND_CHANNEL = 2      // solenoid channel
        const val B_RETRACT_CHANNEL = 3// solenoid channel
    }

    object Intake {
        const val Left_Intake_Motor = 13
        const val Right_Intake_Motor = 14
    }

    object Launcher {
        const val Launcher_Left_Motor = 15
        const val Launcher_Right_Motor = 16
    }

    object Mecanum {
        const val FRONT_LEFT_ID = 9
        const val FRONT_RIGHT_ID = 10
        const val REAR_LEFT_ID = 11
        const val REAR_RIGHT_ID = 12
    }

    object Swerve {
        const val FRONT_LEFT_DRIVING_ID = 3
        const val FRONT_LEFT_TURNING_ID = 4

        const val FRONT_RIGHT_DRIVING_ID = 1
        const val FRONT_RIGHT_TURNING_ID = 2

        const val REAR_LEFT_DRIVING_ID = 7
        const val REAR_LEFT_TURNING_ID = 8

        const val REAR_RIGHT_DRIVING_ID = 5
        const val REAR_RIGHT_TURNING_ID = 6
    }
}