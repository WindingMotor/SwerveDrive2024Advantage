package frc.robot.wmlib2.commands

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.wmlib2.swerve.Swerve

class SwervePathplanner(
        trajectoryName: String,
        swerve: Swerve
): SequentialCommandGroup(){

    init{
        addRequirements(swerve)

        addCommands(
                swerve.followPathPlanner(trajectoryName)

        )

    }


}

