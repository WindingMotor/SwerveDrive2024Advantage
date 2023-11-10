
package frc.robot.wmlib2.sensor

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface IO_Gyro{

    @AutoLog
    class GyroIOInputs{
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
    }

    fun updateInputs(inputs: GyroIOInputs){}

}
