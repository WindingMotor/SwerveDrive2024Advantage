
package frc.robot

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
        val moduleName: String){

        FRONTLEFT(1, 2, 3, 4.0, "FrontLeft"),
        FRONTRIGHT(1, 2, 3, 4.0, "BackRight"),
        BACKLEFT(1, 2, 3, 4.0, "BackLeft"),
        BACKRIGHT(1, 2, 3, 4.0, "BackRight")
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
    
}
