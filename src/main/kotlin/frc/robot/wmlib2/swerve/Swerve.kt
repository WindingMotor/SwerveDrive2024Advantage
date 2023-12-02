

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

class Swerve(
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

    private var setpoint = ChassisSpeeds()
    private var lastSetpointStates = Array(4){ SwerveModuleState() }

    val TRACK_WIDTH = 0.53975 // Distance between RIGHT and LEFT wheel centers
    val WHEEL_BASE = 0.53975; // Distance between FRONT and BACK wheel centers

    val kinematics = SwerveDriveKinematics(
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
        Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Right
        Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Back Left
        Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Back Right
    )

    // POSE ESTIMATOR GOES HERE 0_0
    private var lastModulePositionsMeters = arrayOf(0.0, 0.0, 0.0, 0.0)

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
    
        }else if(DriverStation.isEnabled()){ // Run swerve modules if robot is enabled.

            val setpointTwist = Pose2d().log(Pose2d(
                setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)
            ))
 
            val adjustedSpeeds = ChassisSpeeds(
                setpointTwist.dx / Constants.loopPeriodSecs,
                setpointTwist.dy / Constants.loopPeriodSecs,
                setpointTwist.dtheta / Constants.loopPeriodSecs
            )

            val setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds)
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5) // Max linear speed in m/s

            if(adjustedSpeeds.vxMetersPerSecond == 0.0 &&
                adjustedSpeeds.vyMetersPerSecond == 0.0 &&
                adjustedSpeeds.omegaRadiansPerSecond == 0.0){
                for(i in 0 until 4){ setpointStates[i] = SwerveModuleState(0.0, lastSetpointStates[i].angle) }
            }

            lastSetpointStates = setpointStates

            val optimizedStates = Array(4){ SwerveModuleState() }
            for(i in 0 until 4){ optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]) }

            // Log the setpoints and optimized setpoints of the modules
            Logger.recordOutput("SwerveStates/Setpoints", setpointStates)
            Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates)    

        }

        // Still in periodic function

        // Log the module states
        val measuredStates = Array(4){ SwerveModuleState() }
        for(i in 0 until 4){ measuredStates[i] = modules[i].getState() }
        Logger.recordOutput("SwerveStates/Measured", measuredStates);

        // Update the robot odometry
        val wheelDeltas = Array(4){ SwerveModulePosition() }
        for(i in 0 until 4){
            wheelDeltas[i] = SwerveModulePosition(
                modules[i].getPositionMeters() - lastModulePositionsMeters[i],
                modules[i].getAngle()
            )
        }
        
    }





}