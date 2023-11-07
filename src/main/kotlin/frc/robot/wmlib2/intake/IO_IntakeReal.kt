
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.subsystems.intake.IO_IntakeBase
import frc.robot.subsystems.intake.IO_IntakeBase.IntakeInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax

// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
class IO_Intake_Real : IO_IntakeBase{

    // Intake motor, sets current limit to 30A and removes any inversion
    private val intakeMotor = CANSparkMax(1, MotorType.kBrushless).apply{ 
        setSmartCurrentLimit(30);
        setInverted(false);
        burnFlash();
    }

    // Intake encoder, example of setting position conversion factor
    private val intakeEncoder = intakeMotor.encoder.apply{ 
        positionConversionFactor = 0.0
    }

    override fun updateInputs(inputs: IntakeInputs){}

    override fun setMotorVoltage(voltage: Double){
    }

    

}