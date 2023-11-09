
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.intake.IO_ModuleBase.ModuleInputs
import com.revrobotics.CANSparkMax.IdleMode

class Module(
    private val io: IO_ModuleBase
): SubsystemBase(){

    private val inputs = IO_ModuleBase.ModuleInputs()

    init{

    }

    override fun periodic(){

        // Update the inputs
        io.updateInputs(inputs)

        // Process inputs and send to logger
        Logger.processInputs("Intake", inputs)

    }

}