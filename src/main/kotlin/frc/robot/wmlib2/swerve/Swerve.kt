

package frc.robot.wmlib2.swerve

import com.pathplanner.lib.commands.FollowPathHolonomic
import com.pathplanner.lib.commands.FollowPathWithEvents
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.wmlib2.sensor.IO_Gyro
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.geometry.Twist3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs


class Swerve(
    frontLeftIO: IO_ModuleBase,
    frontRightIO: IO_ModuleBase,
    backLeftIO: IO_ModuleBase,
    backRightIO: IO_ModuleBase,
    private val gyroIO: IO_Gyro
): SubsystemBase(){


    val gyroInputs = IO_Gyro.GyroIOInputs()

    private val frontLeftModule = Module(frontLeftIO, Constants.ModuleSettings.FRONTLEFT)
    private val frontRightModule = Module(frontRightIO, Constants.ModuleSettings.FRONTRIGHT)
    private val backLeftModule = Module(backLeftIO, Constants.ModuleSettings.BACKLEFT)
    private val backRightModule = Module(backRightIO, Constants.ModuleSettings.BACKRIGHT)

    private val modules = mutableListOf(frontLeftModule, frontRightModule, backLeftModule, backRightModule)

    private var setpointSpeeds = ChassisSpeeds()

    // *testSetpoint.toTypedArray()

    // POSE ESTIMATOR GOES HERE 0_0
    private var lastModulePositionsMeters = arrayOf(0.0, 0.0, 0.0, 0.0)

    // Non-vision odometry test
    private val traditionalOdometry = SwerveDriveOdometry(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition, getSwerveModulePositions().toTypedArray())
    override fun periodic(){

        // Update gyroscope inputs.
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Gyroscope", gyroInputs)

        // Update modules periodic loop.
        for(module in modules){
            module.periodic()
        }


        // Get the swerve module states from kinematics method using current setpointSpeeds
        val states = Constants.Kinematics.KINEMATICS.toSwerveModuleStates(setpointSpeeds)

        // Desaturate the states, make every turn position and velocity is possible.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Kinematics.MAX_LINEAR_VELOCITY)

        // Set each swerve module with its respective state
        for(index in modules.indices) {
            modules[index].runWithState(states[index])
        }

        // Record the setpoint states
        Logger.recordOutput("SwerveStates/setpoints", *states)

        // Record the current, "real", states
        val measuredStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
        for(i in 0 until 4){ measuredStates[i] = modules[i].getModuleState() }
        Logger.recordOutput("SwerveStates/Measured", *measuredStates.toTypedArray())


        // Log the current robot pose in 3d, 2d, and 2d traditional
        Logger.recordOutput("Odometry/estimatedPose", Pose2d(getRobotEstimatedPose().x, getRobotEstimatedPose().y, getRobotEstimatedPose().rotation))

        Logger.recordOutput("Odometry/traditionalPose", Pose2d(
                Translation2d(traditionalOdometry.poseMeters.x, traditionalOdometry.poseMeters.y),
                Rotation2d(traditionalOdometry.poseMeters.rotation.radians))
        )




    }

    // Set the swerve setpoint to the desired chassis speeds
    fun runWithSpeeds(newSpeeds: ChassisSpeeds){
        setpointSpeeds = newSpeeds

    }


    // Stop swerve modules
    fun stop(){
        runWithSpeeds(ChassisSpeeds())
    }

    fun getSwerveModulePositions(): MutableList<SwerveModulePosition>{
        val positions = mutableListOf<SwerveModulePosition>()

        for (module in modules) {
            positions.add(module.getModulePosition())
        }
        return positions
    }

    fun getRobotEstimatedPose(): Pose2d = Pose2d(Translation2d(traditionalOdometry.poseMeters.x, traditionalOdometry.poseMeters.y), getRobotEstimatedRotation())

    fun getRobotEstimatedRotation(): Rotation2d = gyroInputs.yawPosition // REPLACE WITH POSE ESTIMATOR

    /*
    fun followPathPlanner(pathName: String): Command {
        val path = PathPlannerPath.fromPathFile(pathName)

        // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
        return FollowPathWithEvents(
                FollowPathHolonomic(
                        path,
                        this::getEstimatedPose, // Robot pose supplier
                        this::setpoint, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        HolonomicPathFollowerConfig(
                                PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                                4.5, // Max module speed, in m/s
                                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                                ReplanningConfig() // Default path replanning config. See the API for the options here
                        ),
                        this // Reference to this subsystem to set requirements
                ),
                path, // FollowPathWithEvents also requires the path
                this::getEstimatedPose // FollowPathWithEvents also requires the robot pose supplier
        )
    }

     */

}


