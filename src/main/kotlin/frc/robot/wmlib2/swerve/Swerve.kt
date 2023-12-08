

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

    private val gyroInputs = IO_Gyro.GyroIOInputs()

    private val frontLeftModule = Module(frontLeftIO, Constants.ModuleSettings.FRONTLEFT)
    private val frontRightModule = Module(frontRightIO, Constants.ModuleSettings.FRONTRIGHT)
    private val backLeftModule = Module(backLeftIO, Constants.ModuleSettings.BACKLEFT)
    private val backRightModule = Module(backRightIO, Constants.ModuleSettings.BACKRIGHT)

    private val modules = mutableListOf(frontLeftModule, frontRightModule, backLeftModule, backRightModule)

    private var setpoint = ChassisSpeeds()
    private var lastSetpointStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())


// *testSetpoint.toTypedArray()

    // POSE ESTIMATOR GOES HERE 0_0
    private var lastModulePositionsMeters = arrayOf(0.0, 0.0, 0.0, 0.0)

    // Non-vision odometry test
    private val traditionalSwerveDriveOdometry = SwerveDriveOdometry(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition, arrayOf(SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition()))

    override fun periodic(){

        // Update gyroscope inputs.
        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Gyroscope", gyroInputs)

        // Update modules periodic loop.
        for(module in modules){
            module.periodic()
        }

        // Stop modules if robot is disabled.
        if(DriverStation.isDisabled()){
            for(module in modules){ module.stop() }

            //Logger.recordOutput("SwerveStates/Setpoints", arrayOfNulls<SwerveModuleState>(0).toMutableList())
            //Logger.recordOutput("SwerveStates/SetpointsOptimized", arrayOfNulls<SwerveModuleState>(0).toMutableList())


        }else if(DriverStation.isEnabled()){ // Run swerve modules if robot is enabled.

            // Get the twist from setpoint
            val setpointTwist = Pose2d().log(Pose2d(
                setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)
            ))

            // Get new ChassisSpeeds from setPoint twist while applying loopPeriodic
            val adjustedSpeeds = ChassisSpeeds(
                setpointTwist.dx / Constants.loopPeriodSecs,
                setpointTwist.dy / Constants.loopPeriodSecs,
                setpointTwist.dtheta / Constants.loopPeriodSecs
            )

            // Convert the adjustedSpeeds to setpointStates
            val setpointStates = Constants.Kinematics.KINEMATICS.toSwerveModuleStates(adjustedSpeeds)

            // Desaturate setpoint states, make sure every position is obtainable
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5) // Max linear speed in m/s

            // If the adjustedSpeeds does not move the robot keep the old setpointStates
            if(adjustedSpeeds.vxMetersPerSecond == 0.0 &&
                adjustedSpeeds.vyMetersPerSecond == 0.0 &&
                adjustedSpeeds.omegaRadiansPerSecond == 0.0){
                for(i in 0 until 4){ setpointStates[i] = SwerveModuleState(0.0, lastSetpointStates[i].angle) }
            }

            // Set the lastSetpointStates to the current ones
            lastSetpointStates = setpointStates.toMutableList()

            // Send the states to each swerve module
            for (i in 0 until 4) {
                modules[i].runSetpoint(setpointStates[i])
            }

            // Log the setpoints of each swerve module
            Logger.recordOutput("SwerveStates/Setpoints", *setpointStates.toMutableList().toTypedArray())
            //Logger.recordOutput("SwerveStates/SetpointsOptimized", *optimizedStates.toTypedArray())
        }

        // Still in the periodic function

        // Get the current module states
        val measuredStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
        for(i in 0 until 4){ measuredStates[i] = modules[i].getState() }
        // Log the current module states
        Logger.recordOutput("SwerveStates/Measured", *measuredStates.toTypedArray())


        // Update the robot odometry

        // Get the current swerveModulePositions, aka wheel deltas
        val wheelDeltas = Array(4){ SwerveModulePosition() }
        for(i in 0 until 4){
            wheelDeltas[i] = SwerveModulePosition(
                modules[i].getPositionMeters() - lastModulePositionsMeters[i],
                modules[i].getAngle()
            )
        }

        // Get twist from the wheel deltas
        val twist = Constants.Kinematics.KINEMATICS.toTwist2d(*wheelDeltas)

        // Get current gyro yaw position
        val gyroYaw = Rotation2d(gyroInputs.yawPosition.radians)

        // Get the current robot pose from estimator
        var robotPose = Pose3d(getEstimatedPose())

        // Update traditional odometry
        traditionalSwerveDriveOdometry.update(gyroInputs.yawPosition, wheelDeltas)

        // IDK what this does, will find out later lol
        robotPose = robotPose.exp(
            Twist3d(
                0.0,
                0.0,
                abs(gyroInputs.pitchPosition.radians) * Constants.Kinematics.TRACK_WIDTH / 2.0,
                0.0,
                gyroInputs.pitchPosition.radians,
                0.0
        )).exp(
            Twist3d(
                0.0,
                0.0,
                abs(gyroInputs.rollPosition.radians) * Constants.Kinematics.TRACK_WIDTH / 2.0,
                gyroInputs.rollPosition.radians,
                0.0,
                0.0
            )
        )

        // Log the current robot pose in 3d, 2d, and 2d traditional
        Logger.recordOutput("Odometry/Robot3d", robotPose)
        Logger.recordOutput("Odometry/Robot2d", Pose2d(robotPose.x, robotPose.y, robotPose.rotation.toRotation2d()))

        Logger.recordOutput("Odometry/Traditional2d", Pose2d(
                Translation2d(traditionalSwerveDriveOdometry.poseMeters.x, traditionalSwerveDriveOdometry.poseMeters.y),
                Rotation2d(traditionalSwerveDriveOdometry.poseMeters.rotation.radians))
        )


        // Get current chassis speeds
        val chassisSpeeds = Constants.Kinematics.KINEMATICS.toChassisSpeeds(*measuredStates.toTypedArray())

        // Get current linear field velocity
        val linearFieldVelocity = Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(getEstimatedRotation())

        // Set current field velocity
        val fieldVelocity = Twist2d(
                linearFieldVelocity.x,
                linearFieldVelocity.y,
                if(gyroInputs.connected) gyroInputs.yawPosition.radians else chassisSpeeds.omegaRadiansPerSecond
        )


    }

    // Set the swerve setpoint to the desired chassis speeds
    fun runVelocity(speeds: ChassisSpeeds){
        setpoint = speeds
    }

    // Put swerve modules in an X pattern to stop movement
    fun stopWithX(){
        stop()
        lastSetpointStates.forEachIndexed { i, state ->
            lastSetpointStates[i] = SwerveModuleState(state.speedMetersPerSecond, Constants.Kinematics.MODULE_TRANSLATIONS[i].angle)
        }
    }

    // Stop swerve modules
    fun stop(){
        runVelocity(ChassisSpeeds())
    }

    // Get the estimated rotation of the robot from vision
    fun getEstimatedRotation(): Rotation2d = gyroInputs.yawPosition // Return pose estimator rotation when in usage: poseEstimator.getLatestPose().getRotation();

    // Get the estimated pose of the robot from vision
    fun getEstimatedPose(): Pose2d = traditionalSwerveDriveOdometry.poseMeters // Return pose estimator position when in usage: poseEstimator.getLatestPose();

    // Assuming this is a method in your drive subsystem
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

}