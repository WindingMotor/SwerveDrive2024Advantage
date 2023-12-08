
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import com.revrobotics.CANSparkMax.IdleMode

// The base interface for using the swerve module abstracts to the real and simulation classes.
interface IO_ModuleBase{
    class ModuleInputs : LoggableInputs{

        var drivePositionRad: Double = 0.0
        var driveVelocityRadPerSec: Double = 0.0
        var driveAppliedPercentage: Double = 0.0
        var driveCurrentAmps: Double = 0.0
        var driveTempCelcius: Double = 0.0
    
        var turnAbsolutePositionRad: Double = 0.0
        var turnPositionRad: Double = 0.0
        var turnVelocityRadPerSec: Double = 0.0
        var turnAppliedPercentage: Double = 0.0
        var turnCurrentAmps: Double = 0.0
        var turnTempCelcius: Double = 0.0

        var turnPIDReference = 0.0
        var drivePIDReference = 0.0

        override fun toLog(table: LogTable){
            table.put("drivePositionRad", drivePositionRad)
            table.put("driveVelocityRadPerSec", driveVelocityRadPerSec)
            table.put("driveAppliedPercentage", driveAppliedPercentage)
            table.put("driveCurrentAmps", driveCurrentAmps)
            table.put("driveTempCelsius", driveTempCelcius)

            table.put("turnAbsolutePositionRad", turnAbsolutePositionRad)
            table.put("turnPositionRad", turnPositionRad)
            table.put("turnVelocityRadPerSec", turnVelocityRadPerSec)
            table.put("turnAppliedPercentage", turnAppliedPercentage)
            table.put("turnCurrentAmps", turnCurrentAmps)
            table.put("turnTempCelsius", turnTempCelcius)

            table.put("turnPIDReference", turnPIDReference)
            table.put("drivePIDReference", drivePIDReference)
        }

        override fun fromLog(table: LogTable){
            drivePositionRad = table.get("drivePositionRad", drivePositionRad)
            driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec)
            driveAppliedPercentage = table.get("driveAppliedVolts", driveAppliedPercentage)
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps)
            driveTempCelcius = table.get("turnTempCelsius", driveTempCelcius)

            turnAbsolutePositionRad = table.get("turnAbsolutePositionRad", turnAbsolutePositionRad)
            turnPositionRad = table.get("turnPositionRad", turnPositionRad)
            turnVelocityRadPerSec = table.get("turnVelocityRadPerSec", turnVelocityRadPerSec)
            turnAppliedPercentage = table.get("turnAppliedPercentage", turnAppliedPercentage)
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps)
            driveTempCelcius = table.get("driveTempCelsius", driveTempCelcius)

            turnPIDReference = table.get("turnPIDReference", turnPIDReference)
            drivePIDReference = table.get("drivePIDReference", drivePIDReference)
        }
    }

    fun updateInputs(inputs: ModuleInputs){}

    fun setDriveOutput(percent: Double){}

    fun setTurnOutput(percent: Double){}

    fun stop(){}

    fun setPIDReferences(driveReference: Double, turnReference: Double){}


}