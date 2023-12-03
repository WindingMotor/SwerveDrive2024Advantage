
package frc.robot
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.Constants.RobotMode
import frc.robot.wmlib2.intake.Intake
import frc.robot.wmlib2.intake.IO_IntakeReal
import frc.robot.wmlib2.intake.IO_IntakeSim
import frc.robot.wmlib2.sensor.IO_Navx
import frc.robot.wmlib2.swerve.IO_ModuleBase
import frc.robot.wmlib2.swerve.IO_ModuleReal
import frc.robot.wmlib2.swerve.Swerve

class RobotContainer {

    val isReal = true

    val intake = Intake(if(isReal) IO_IntakeReal() else IO_IntakeSim())

    val swerve = Swerve(
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.FRONTLEFT) else IO_ModuleReal(Constants.ModuleSettings.FRONTLEFT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.FRONTRIGHT) else IO_ModuleReal(Constants.ModuleSettings.FRONTRIGHT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.BACKLEFT) else IO_ModuleReal(Constants.ModuleSettings.BACKLEFT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.BACKRIGHT) else IO_ModuleReal(Constants.ModuleSettings.BACKRIGHT),
            if(isReal) IO_Navx() else IO_Navx()
    )

    init{
        configureBindings()
    }

    private fun configureBindings(){

    }

    val autonomousCommand: Command? = null

}
