
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import com.revrobotics.CANSparkMax.IdleMode

// The base interface for using the intake abstracts to the real and simulation classes.
interface IO_IntakeBase{
    class IntakeInputs : LoggableInputs{

        var appliedVoltage = 0.0
        var motorTemperature = 0.0
        var encoderVelocity = 0.0

        override fun toLog(table: LogTable){
            table.put("appliedVoltage", appliedVoltage)
            table.put("motorTemperature", motorTemperature)
            table.put("encoderVelocity", encoderVelocity)
        }

        override fun fromLog(table: LogTable){
            appliedVoltage = table.get("motorSpeed", appliedVoltage)
            motorTemperature = table.get("motorTemperature", motorTemperature)
            encoderVelocity = table.get("encoderVelocity", encoderVelocity)
          }

    }

    fun updateInputs(inputs: IntakeInputs){}

    fun setMotorVoltage(voltage: Double){}

    fun setMotorIdleMode(mode: IdleMode)


}