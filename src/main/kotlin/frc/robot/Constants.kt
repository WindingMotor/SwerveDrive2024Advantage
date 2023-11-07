
package frc.robot

 object Constants{

    // Current robot mode.
    val currentMode: Mode = Mode.REAL

    enum class Mode{
        // Running on a real robot.
        REAL,
        // Running a physics simulator.
        SIM,
        // Replaying from a log file.
        REPLAY
    }

}
