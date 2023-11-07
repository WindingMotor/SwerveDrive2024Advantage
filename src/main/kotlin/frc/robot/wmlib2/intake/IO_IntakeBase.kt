
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable

// The base interface for using the intake abstracts to the real and simulation classes.
interface IO_IntakeBase{
    class IntakeInputs : LoggableInputs{

        var motorVoltage = 0.0
        var motorTemperature = 0.0
        var motorVelocity = 0.0

        override fun toLog(table: LogTable){
            table.put("motorVoltage/r", motorVoltage)
            table.put("motorTemperature/g", motorTemperature)
            table.put("motorVelocity/b", motorVelocity)
        }

        override fun fromLog(table: LogTable){
            motorVoltage = table.get("motorVoltage/r", motorVoltage)
            motorTemperature = table.get("motorTemperature/b", motorTemperature)
            motorVelocity = table.get("motorVelocity/b", motorVelocity)
          }

    }

    fun updateInputs(inputs: IntakeInputs){}

    fun setMotorVoltage(voltage: Double){}



}