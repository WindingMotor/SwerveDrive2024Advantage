
package frc.robot

object Constants{

    // Current robot mode.
    val currentMode: RobotMode = RobotMode.REAL

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
        val absoluteEncoderOffset: Double){

        FRONTLEFT(1, 2, 3, 4.0),
        FRONTRIGHT(1, 2, 3, 4.0),
        BACKLEFT(1, 2, 3, 4.0),
        BACKRIGHT(1, 2, 3, 4.0)

        
    }

}
