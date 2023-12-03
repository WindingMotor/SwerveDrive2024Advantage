

package frc.robot.wmlib2.swerve

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.wmlib2.sensor.IO_Gyro
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import kotlin.math.abs

class Swerve(
    private val frontLeftIO: IO_ModuleBase,
    private val frontRightIO: IO_ModuleBase,
    private val backLeftIO: IO_ModuleBase,
    private val backRightIO: IO_ModuleBase,
    private val gyroIO: IO_Gyro
): SubsystemBase(){

    private val gyroInputs = IO_Gyro.GyroIOInputs()

    private val frontLeftModule = Module(frontLeftIO, Constants.ModuleSettings.FRONTLEFT)
    private val frontRightModule = Module(frontRightIO, Constants.ModuleSettings.FRONTRIGHT)
    private val backLeftModule = Module(backLeftIO, Constants.ModuleSettings.BACKLEFT)
    private val backRightModule = Module(backRightIO, Constants.ModuleSettings.BACKRIGHT)

    private val modules = arrayOf(frontLeftModule, frontRightModule, backLeftModule, backRightModule)

    private var setpoint = ChassisSpeeds()
    private var lastSetpointStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())


// *testSetpoint.toTypedArray()
    val TRACK_WIDTH = 0.53975 // Distance between RIGHT and LEFT wheel centers
    val WHEEL_BASE = 0.53975; // Distance between FRONT and BACK wheel centers

    val moduleTranslations = arrayOf(
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
        Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Right
        Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Back Left
        Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Back Right
    )

    val kinematics = SwerveDriveKinematics(*moduleTranslations)

    // POSE ESTIMATOR GOES HERE 0_0
    private var lastModulePositionsMeters = arrayOf(0.0, 0.0, 0.0, 0.0)

    // Non-vision odometry test
    val traditionalSwerveDriveOdometry = SwerveDriveOdometry(kinematics, gyroInputs.yawPosition, arrayOf())

    init{

    }

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

            // Get the twist from the setpoint
            val setpointTwist = Pose2d().log(Pose2d(
                setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)
            ))

            // Adjust the speeds using setpointTwist
            val adjustedSpeeds = ChassisSpeeds(
                setpointTwist.dx / Constants.loopPeriodSecs,
                setpointTwist.dy / Constants.loopPeriodSecs,
                setpointTwist.dtheta / Constants.loopPeriodSecs
            )

            // Convert speeds into swerveModuleStates
            val setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds)
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5) // Max linear speed in m/s

            // If the new states do nothing keep the old ones
            if(adjustedSpeeds.vxMetersPerSecond == 0.0 &&
                adjustedSpeeds.vyMetersPerSecond == 0.0 &&
                adjustedSpeeds.omegaRadiansPerSecond == 0.0){
                for(i in 0 until 4){ setpointStates[i] = SwerveModuleState(0.0, lastSetpointStates[i].angle) }
            }

            // Set the last setpoints to the current ones
            lastSetpointStates = setpointStates.toMutableList()

            // Optimize the states then send them to the swerve modules
            val optimizedStates = mutableListOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
            for(i in 0 until 4){ optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]) }

            // Log the setpoints and optimized setpoints of the modules
            Logger.recordOutput("SwerveStates/Setpoints", *setpointStates.toMutableList().toTypedArray())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", *optimizedStates.toTypedArray())
        }

        // Log a test odometry pose
        Logger.recordOutput("Odometry/testPose", Pose2d(Translation2d(3.0,3.0), Rotation2d()))

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
        val twist = kinematics.toTwist2d(*wheelDeltas)

        // Get current gyro yaw position
        val gyroYaw = Rotation2d(gyroInputs.yawPosition.radians)

        // Get the current robot pose from estimator
        var robotPose = Pose3d(getEstimatedPose())

        // Update traditional odometry
        traditionalSwerveDriveOdometry.update(gyroInputs.yawPosition, wheelDeltas)

        // uhmmm....
        robotPose = robotPose.exp(
            Twist3d(
                0.0,
                0.0,
                abs(gyroInputs.pitchPosition.radians) * TRACK_WIDTH / 2.0,
                0.0,
                gyroInputs.pitchPosition.radians,
                0.0
        )).exp(
            Twist3d(
                0.0,
                0.0,
                abs(gyroInputs.rollPosition.radians) * TRACK_WIDTH / 2.0,
                gyroInputs.rollPosition.radians,
                0.0,
                0.0
            )
        )

        // Log the current robot pose
        Logger.recordOutput("Odometry/Robot3d", robotPose)

        // Get current chassis speeds
        val chassisSpeeds = kinematics.toChassisSpeeds(*measuredStates.toTypedArray())

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
            lastSetpointStates[i] = SwerveModuleState(state.speedMetersPerSecond, moduleTranslations[i].angle)
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

}