package frc.robot.wmlib2.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.wmlib2.swerve.Swerve
import frc.robot.wmlib2.util.GeomUtil
import kotlin.math.hypot
import kotlin.math.withSign

class SwerveChoreo(
        private val trajectoryName: String,

        private val swerve: Swerve
): Command(){

    init{
        addRequirements(swerve)
    }

   // val traj = Choreo.getTrajectory("Trajectory");


    override fun initialize() {


    }

    override fun execute(){

        // Get current inputs from controller
        val xCurrent = 0.0
        val yCurrent = 0.0
        val rCurrent = 0.0

        // Get direction and magnitude of linear axes (x & y) & rotation, so we can modify them
        var linearMagnitude = hypot(xCurrent, yCurrent)
        val linearDirection = Rotation2d(xCurrent, yCurrent)
        var angularSpeed = rCurrent

        // Apply deadband, for mitigating controller drift
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, Constants.Kinematics.DRIVE_DEADBAND)
        angularSpeed = MathUtil.applyDeadband(angularSpeed, Constants.Kinematics.DRIVE_DEADBAND)

        // Apply squaring, the farther the driver pushes the faster the robot goes
        linearMagnitude = (linearMagnitude * linearMagnitude).withSign(linearMagnitude)
        angularSpeed = (angularSpeed * angularSpeed).withSign(angularSpeed)

        // Apply speed limits, make sure robot doesn't go vroom, vroom to much
        linearMagnitude *= Constants.Kinematics.LINEAR_SPEED_LIMIT
        angularSpeed *= Constants.Kinematics.ANGULAR_SPEED_LIMIT

        // Calculate new linear components from linearMagnitude and linearDirection
        val linearVelocity: Translation2d = Pose2d(Translation2d(), linearDirection).transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0)).translation

        // Create speeds and convert linearVelocity to m/s
        var speeds = ChassisSpeeds(
            linearVelocity.x * Constants.Kinematics.MAX_LINEAR_VELOCITY,
            linearVelocity.y * Constants.Kinematics.MAX_LINEAR_VELOCITY,
            angularSpeed * Constants.Kinematics.MIN_EXTENSION_ANGULAR_VELOCITY
        )

        // Flip robot rotation if on different alliance, field relative
        var currentRotation = swerve.getEstimatedRotation()
        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red){ // If there is not an option for the Optional<>! it will go to Blue
            currentRotation = currentRotation.plus(Rotation2d(Math.PI))
        }

        // Apply field relative or non-field relative for PathPlanner
        speeds = if(false){
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    currentRotation
            )
        }else{
            ChassisSpeeds.fromRobotRelativeSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    currentRotation
            )
        }

        // Get current drive translation from vision, flip if need be
        //val driveTranslation = AllianceFlipUtil.apply(swerve.getEstimatedPose().translation)

        swerve.runVelocity(speeds)

        /*

        // Made modules go in X pattern if the robot is on the charge station
        if (speeds.vxMetersPerSecond.absoluteValue < 1e-3
                && speeds.vyMetersPerSecond.absoluteValue < 1e-3
                && speeds.omegaRadiansPerSecond.absoluteValue < 1e-3
                && driveTranslation.x > FieldConstants.Community.chargingStationInnerX
                && driveTranslation.x < FieldConstants.Community.chargingStationOuterX
                && driveTranslation.y > FieldConstants.Community.chargingStationRightY
                && driveTranslation.y < FieldConstants.Community.chargingStationLeftY
        ){
            swerve.stopWithX()
        }else{
            swerve.runVelocity(speeds)
        }
        */

    }

    override fun end(interrupted: Boolean){
        swerve.stop()
    }

}