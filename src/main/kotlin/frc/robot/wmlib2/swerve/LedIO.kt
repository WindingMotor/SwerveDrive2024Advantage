
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import edu.wpi.first.math.geometry.Rotation2d

// Main interface
interface LedIO{
  // Input Class
  class LedIOInputs : LoggableInputs{

  var r = 0.0
  var g = 0.0
  var b = 0.0

  // Send class values to log
  override fun toLog(table: LogTable){
    table.put("leds/r", r)
    table.put("leds/g", g)
    table.put("leds/b", b)
  }

  // Retrive class valuesf from log
  override fun fromLog(table: LogTable){
    r = table.get("leds/r", r)
    g = table.get("leds/b", g)
    b = table.get("leds/b", b)
  }

}
  // Interface methods

  fun updateInputs(inputs: LedIOInputs) {}

  fun otherMethod(number: Double) {}

}




