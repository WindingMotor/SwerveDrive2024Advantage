

package frc.robot.wmlib2.swerve

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.ModuleSettings
import frc.robot.wmlib2.sensor.IO_Gyro
import frc.robot.wmlib2.sensor.IO_Navx
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer


class SwerveDrive(
    private val frontLeftIO: IO_ModuleBase,
    private val frontRightIO: IO_ModuleBase,
    private val backLeftIO: IO_ModuleBase,
    private val backrightIO: IO_ModuleBase,
    private val gyroIO: IO_Gyro
): SubsystemBase(){

    //private val gyroInputs = IO_Gyro.GyroIOInputs()

    private val frontLeftModule = Module(frontLeftIO, Constants.ModuleSettings.FRONTLEFT)
    private val frontRightModule = Module(frontRightIO, Constants.ModuleSettings.FRONTRIGHT)
    private val backLeftModule = Module(backLeftIO, Constants.ModuleSettings.BACKLEFT)
    private val backRightModule = Module(backrightIO, Constants.ModuleSettings.BACKRIGHT)

    private val modules = arrayOf(frontLeftModule, frontRightModule, backLeftModule, backRightModule)

    init{

    }

    override fun periodic(){

        // Update gyroscope inputs.
        // gyroIO.updateInputs(gyroInputs)
        // Logger.processInputs("Gyroscope", gyroInputs)

        // Update modules periodic loop.
        for(module in modules){
            module.periodic()
        }

        // Stop modules if robot is disabled.
        if(DriverStation.isDisabled()){ for(module in modules){ module.stop() }   
    
        }else if(DriverStation.isEnabled()){ // Run swerve modules.

            val setpointTwist = Pose2d().log(Pose2d(
                setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)))

            val adjustedSpeeds = ChassisSpeeds(
                setpointTwist.dx / Constants.loopPeriodSecs,
                setpointTwist.dy / Constants.loopPeriodSecs,
                setpointTwist.dtheta / Constants.loopPeriodSecs

        }

    }

}