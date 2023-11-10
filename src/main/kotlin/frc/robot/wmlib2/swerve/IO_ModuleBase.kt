
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import com.revrobotics.CANSparkMax.IdleMode

// The base interface for using the swerve module abstracts to the real and simulation classes.
interface IO_ModuleBase{
    class ModuleInputs : LoggableInputs{

        var drivePositionRad: Double = 0.0
        var driveVelocityRadPerSec: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveCurrentAmps: Double = 0.0
        var driveTempCelcius: Double = 0.0
    
        var turnAbsolutePositionRad: Double = 0.0
        var turnPositionRad: Double = 0.0
        var turnVelocityRadPerSec: Double = 0.0
        var turnAppliedVolts: Double = 0.0
        var turnCurrentAmps: Double = 0.0
        var turnTempCelcius: Double = 0.0

        override fun toLog(table: LogTable){
            table.put("drivePositionRad", drivePositionRad)
            table.put("driveVelocityRadPerSec", driveVelocityRadPerSec)
            table.put("driveAppliedVolts", driveAppliedVolts)
            table.put("driveCurrentAmps", driveCurrentAmps)
            table.put("driveTempCelcius", driveTempCelcius)

            table.put("turnAbsolutePositionRad", turnAbsolutePositionRad)
            table.put("turnPositionRad", turnPositionRad)
            table.put("turnVelocityRadPerSec", turnVelocityRadPerSec)
            table.put("turnAppliedVolts", turnAppliedVolts)
            table.put("turnCurrentAmps", turnCurrentAmps)
            table.put("turnTempCelcius", turnTempCelcius)
        }

        override fun fromLog(table: LogTable){
            drivePositionRad = table.get("drivePositionRad", drivePositionRad)
            driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec)
            driveAppliedVolts = table.get("driveAppliedVolts", driveAppliedVolts)
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps)
            driveTempCelcius = table.get("driveTempCelcius", driveTempCelcius)

            turnAbsolutePositionRad = table.get("turnAbsolutePositionRad", turnAbsolutePositionRad)
            turnPositionRad = table.get("turnPositionRad", turnPositionRad)
            turnVelocityRadPerSec = table.get("turnVelocityRadPerSec", turnVelocityRadPerSec)
            turnAppliedVolts = table.get("turnAppliedVolts", turnAppliedVolts)
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps)
            driveTempCelcius = table.get("driveTempCelcius", driveTempCelcius)
        }
    }

    fun updateInputs(inputs: ModuleInputs){}

    fun setDriveVoltage(voltage: Double){}

    fun setTurnVoltage(voltage: Double){}

    fun stop(){}
    


}