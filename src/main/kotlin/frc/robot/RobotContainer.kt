
package frc.robot
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.Constants.RobotMode
import frc.robot.wmlib2.intake.Intake
import frc.robot.wmlib2.intake.IO_IntakeReal
import frc.robot.wmlib2.intake.IO_IntakeSim

class RobotContainer {

    val isReal = true

    val intake = Intake(if(isReal){IO_IntakeReal()}else{IO_IntakeSim()})

    init{
        configureBindings()
    }

    private fun configureBindings(){

    }

    val autonomousCommand: Command? = null

}
