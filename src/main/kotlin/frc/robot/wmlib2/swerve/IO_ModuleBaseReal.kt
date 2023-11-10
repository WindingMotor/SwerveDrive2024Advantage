
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.wmlib2.swerve.IO_ModuleBase
import frc.robot.wmlib2.swerve.IO_ModuleBase.ModuleInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax
import com.revrobotics.AbsoluteEncoder
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.Constants
import frc.robot.Constants.MK4SDS
import frc.robot.Constants.ModuleSettings

// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
class IO_ModuleReal(val settings: ModuleSettings) : IO_ModuleBase{

    private val driveMotor = CANSparkMax(settings.driveID, MotorType.kBrushless).apply{
        
        setIdleMode(IdleMode.kCoast)
        setSmartCurrentLimit(45);
        setInverted(false);
        burnFlash();
    }

    private val driveEncoder = driveMotor.encoder.apply{ 
        positionConversionFactor =  Constants.MK4SDS.DRIVE_ROT_2_METER
        velocityConversionFactor = Constants.MK4SDS.DRIVE_RPM_2_MPS
    }

    private val turnMotor = CANSparkMax(settings.turnID, MotorType.kBrushless).apply{
        setIdleMode(IdleMode.kCoast)
        setSmartCurrentLimit(25);
        setInverted(false);
        burnFlash();
    }

    private val turnEncoder = turnMotor.encoder.apply{ 
        positionConversionFactor =  Constants.MK4SDS.TURN_ROT_2_RAD
        velocityConversionFactor = Constants.MK4SDS.TURN_RPM_2_RADPS
    }

    private val turnAbsoluteEncoder = DutyCycleEncoder(settings.absoluteEncoderID).apply{
         setDutyCycleRange(1.0/4096.0, 4095.0/4096.0)
    }

    // Update inputs with current values
    override fun updateInputs(inputs: ModuleInputs){
        inputs.drivePositionRad = driveEncoder.position
        inputs.driveVelocityRadPerSec = driveEncoder.velocity
        inputs.driveAppliedVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveCurrentAmps = driveMotor.outputCurrent
        inputs.driveTempCelcius = driveMotor.motorTemperature

        inputs.turnAbsolutePositionRad = Math.toDegrees(turnAbsoluteEncoder.absolutePosition)
        inputs.turnPositionRad = turnEncoder.position
        inputs.turnVelocityRadPerSec = turnEncoder.velocity
        inputs.turnAppliedVolts = turnMotor.appliedOutput * turnMotor.busVoltage
        inputs.turnCurrentAmps = turnMotor.outputCurrent
        inputs.turnTempCelcius = turnMotor.motorTemperature
    }

    override fun setDriveVoltage(voltage: Double){
        driveMotor.setVoltage(voltage)
    }

    override fun setTurnVoltage(voltage: Double){
        turnMotor.setVoltage(voltage)
    }

    override fun stop(){
        driveMotor.setVoltage(0.0)
        turnMotor.setVoltage(0.0)
    }

}