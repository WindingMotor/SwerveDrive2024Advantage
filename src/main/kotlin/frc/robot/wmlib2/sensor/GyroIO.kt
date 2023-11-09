
package frc.robot.subsystems.sensor

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface GyroIO{

    @AutoLog
    class GyroIOInputs{
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
    }

    fun updateInputs(inputs: GyroIOInputs){}

}
