
package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

object Constants{

    // Current robot mode.
    val currentMode: RobotMode = RobotMode.REAL
    val loopPeriodSecs = 0.02;

    enum class RobotMode{
        // Running on a real robot.
        REAL,
        // Running a physics simulator.
        SIM,
        // Replaying from a log file.
        REPLAY
    }

    enum class ModuleSettings(
        val driveID: Int, val turnID: Int,
        val absoluteEncoderID: Int,
        val absoluteEncoderOffset: Double,
        val moduleName: String
    ){

        FRONTLEFT(1, 2, 0, 1.65, "FrontLeft"),
        FRONTRIGHT(3, 4, 1, 1.39, "FrontRight"),
        BACKLEFT(7, 8, 3, 1.8, "BackLeft"),
        BACKRIGHT(5, 6, 2, -0.12, "BackRight"),

        DEFAULT(26, 27, 32, 32.0, "DEFAULT")
    }

    object MK4SDS{

        const val WHEEL_DIAMETER = 0.1016 // 4 in
        const val DRIVE_GEAR_RATIO = 1 / 6.12 // L3
        const val TURN_GEAR_RATIO = 1 / 12.8
    
        const val DRIVE_ROT_2_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER
        const val TURN_ROT_2_RAD = TURN_GEAR_RATIO * 2 * Math.PI
    
        const val DRIVE_RPM_2_MPS = DRIVE_ROT_2_METER / 60;
        const val TURN_RPM_2_RADPS = TURN_ROT_2_RAD / 60;
      
        const val FREE_MOTOR_SPEED_RPS = 5676 / 60

    }

    object Kinematics{

        const val TRACK_WIDTH = 0.53975 // Distance between RIGHT and LEFT wheel centers
        const val WHEEL_BASE = 0.53975; // Distance between FRONT and BACK wheel centers

        val MODULE_TRANSLATIONS = arrayOf(
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Right
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Back Left
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Back Right
        )

        val KINEMATICS = SwerveDriveKinematics(*MODULE_TRANSLATIONS)

        // Used for conversion to M/S
        const val MAX_LINEAR_VELOCITY = 12.0
        const val MIN_EXTENSION_ANGULAR_VELOCITY = 12.0

        // Joystick speed limits
        const val LINEAR_SPEED_LIMIT = 12.0
        const val ANGULAR_SPEED_LIMIT = 12.0

        const val DRIVE_DEADBAND = 0.05

    }

    enum class DriverBindings(
        val xInput: Int,
        val xInverted: Boolean,
        val yInput: Int,
        val yInverted: Boolean,
        val rInput: Int,
        val rInverted: Boolean,
    ){
        TX16S(0, false,1, false,3, false),
        XBOX(1, true,0, true,4, false)
    }

    
}
