
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.intake.IO_IntakeBase.IntakeInputs
import com.revrobotics.CANSparkMax.IdleMode

class Intake(
    private val io: IO_IntakeBase
): SubsystemBase(){

    // Intake inputs
    private val inputs = IO_IntakeBase.IntakeInputs()

    // Usage showing how to run a method from the io.
    init{
        io.setMotorIdleMode(IdleMode.kCoast)
    }

    override fun periodic(){

        // Update the inputs
        io.updateInputs(inputs)

        // Process inputs and send to logger
        Logger.processInputs("Intake", inputs)

    }

}