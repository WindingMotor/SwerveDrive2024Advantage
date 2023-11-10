
package frc.robot.wmlib2.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.wmlib2.intake.IO_IntakeBase
import frc.robot.wmlib2.intake.IO_IntakeBase.IntakeInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax

// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
class IO_IntakeReal : IO_IntakeBase{

    // Intake motor, sets current limit to 30A and removes any inversion
    private val motor = CANSparkMax(1, MotorType.kBrushless).apply{
        setIdleMode(IdleMode.kCoast)
        setSmartCurrentLimit(30);
        setInverted(false);
        burnFlash();
    }

    // Intake encoder, example of setting position conversion factor
    private val encoder = motor.encoder.apply{ 
        positionConversionFactor = 0.0
    }

    // Update inputs with current values
    override fun updateInputs(inputs: IntakeInputs){
        inputs.appliedVoltage = motor.appliedOutput
        inputs.motorTemperature = motor.motorTemperature
        inputs.encoderVelocity = encoder.velocity
    }

    // Set motor voltage 
    override fun setMotorVoltage(voltage: Double){
        motor.setVoltage(voltage)
    }

    // Set motor idle mode
    override fun setMotorIdleMode(mode: IdleMode){
        motor.setIdleMode(mode)
    }

}