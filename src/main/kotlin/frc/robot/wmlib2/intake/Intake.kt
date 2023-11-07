
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

class Intake(
    private val io: IntakeIO
): SubsystemBase(){

    private val inputs = IntakeIO.IntakeIOInputs()

    init{

    }

    override fun periodic(){
        io.updateInputs(inputs)
        Logger.processInputs(key, inputs)
    }

}