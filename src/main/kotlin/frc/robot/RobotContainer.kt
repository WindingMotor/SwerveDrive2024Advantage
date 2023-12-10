
package frc.robot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.Constants.RobotMode
import frc.robot.wmlib2.commands.SwerveJoystick
import frc.robot.wmlib2.intake.Intake
import frc.robot.wmlib2.intake.IO_IntakeReal
import frc.robot.wmlib2.intake.IO_IntakeSim
import frc.robot.wmlib2.sensor.IO_GyroReal
import frc.robot.wmlib2.sensor.IO_GyroSim
import frc.robot.wmlib2.swerve.IO_ModuleReal
import frc.robot.wmlib2.swerve.IO_ModuleSim
import frc.robot.wmlib2.swerve.Swerve

class RobotContainer() {


    private val isReal = if(Constants.currentMode == RobotMode.REAL) true else false

    private val driverController = CommandXboxController(3)

    //private val intake = Intake(if(isReal) IO_IntakeReal() else IO_IntakeSim())


    private val swerve = Swerve(
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.FRONTLEFT) else IO_ModuleSim(Constants.ModuleSettings.FRONTLEFT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.FRONTRIGHT) else IO_ModuleSim(Constants.ModuleSettings.FRONTRIGHT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.BACKLEFT) else IO_ModuleSim(Constants.ModuleSettings.BACKLEFT),
            if(isReal) IO_ModuleReal(Constants.ModuleSettings.BACKRIGHT) else IO_ModuleSim(Constants.ModuleSettings.BACKRIGHT),
            if(isReal) IO_GyroReal() else IO_GyroSim()
    )


    init{

        configureBindings()

        val driverBindings = Constants.DriverBindings.XBOX

        

        swerve.defaultCommand = SwerveJoystick(
            { if(driverBindings.xInverted) -driverController.getRawAxis(driverBindings.xInput) else driverController.getRawAxis(driverBindings.xInput) },
            { if(driverBindings.yInverted) -driverController.getRawAxis(driverBindings.yInput) else driverController.getRawAxis(driverBindings.yInput) },
            { if(driverBindings.rInverted) -driverController.getRawAxis(driverBindings.rInput) else driverController.getRawAxis(driverBindings.rInput) },
            { true },
            swerve
        )


    }

    private fun configureBindings(){

    }

    val autonomousCommand: Command = PrintCommand("Test")

}
