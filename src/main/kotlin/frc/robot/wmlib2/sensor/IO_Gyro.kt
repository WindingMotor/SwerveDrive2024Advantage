
package frc.robot.wmlib2.sensor

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IO_Gyro{

    class GyroIOInputs: LoggableInputs {
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
        var pitchPosition: Rotation2d = Rotation2d()
        var rollPosition: Rotation2d = Rotation2d()

        override fun toLog(table: LogTable){
            table.put("connected", connected)
            table.put("yawPosition", yawPosition)
            table.put("pitchPosition", pitchPosition)
            table.put("rollPosition", rollPosition)
        }
        override fun fromLog(table: LogTable){
            connected = table.get("connected", connected)
            yawPosition = table.get("yawPosition", yawPosition)
            pitchPosition = table.get("encoderVelocity", pitchPosition)
            rollPosition = table.get("encoderVelocity", rollPosition)
        }
    }

    fun updateInputs(inputs: GyroIOInputs){}

}
