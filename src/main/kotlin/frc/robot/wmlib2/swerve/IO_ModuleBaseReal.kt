
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.subsystems.intake.IO_ModuleBase
import frc.robot.subsystems.intake.IO_ModuleBase.ModuleInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax
import com.revrobotics.AbsoluteEncoder
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.Constants
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
        positionConversionFactor = 0.0
    }

    private val turnMotor = CANSparkMax(settings.turnID, MotorType.kBrushless).apply{
        setIdleMode(IdleMode.kCoast)
        setSmartCurrentLimit(25);
        setInverted(false);
        burnFlash();
    }

    private val turnEncoder = turnMotor.encoder.apply{ 
        positionConversionFactor = 0.0
    }

    private val turnAbsoluteEncoder = DutyCycleEncoder(settings.absoluteEncoderID).apply{
         setDutyCycleRange(1.0/4096.0, 4095.0/4096.0)
    }

    fun turnAbsoluteEncoderRadians: Double = (( 1.0 - turnAbsoluteEncoder.getAbsolutePosition()) * (2.0 * Math.PI)) - settings.absoluteEncoderOffset

    // Update inputs with current values
    override fun updateInputs(inputs: ModuleInputs){
        inputs.drivePositionRad = driveEncoder.position
        inputs.driveVelocityRadPerSec = driveEncoder.velocity
        inputs.driveAppliedVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveCurrentAmps = driveMotor.outputCurrent
        inputs.driveTempCelcius = driveMotor.motorTemperature

        inputs.turnAbsolutePositionRad = turnAbsoluteEncoderRadians()
        inputs.turnPositionRad = turnEncoder.position
        inputs.turnVelocityRadPerSec = turnEncoder.velocity
        inputs.turnAppliedVolts = turnMotor.appliedOutput * turnMotor.busVoltage
        inputs.turnCurrentAmps = turnMotor.outputCurrent
        inputs.turnTempCelcius = turnMotor.motorTemperature
    }

    override fun setDriveVoltage(voltage: Double){}

    override fun setTurnVoltage(voltage: Double){}

    override fun stop(){}

}