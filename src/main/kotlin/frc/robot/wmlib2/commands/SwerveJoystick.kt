package frc.robot.wmlib2.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.wmlib2.swerve.Swerve
import frc.robot.wmlib2.util.AllianceFlipUtil
import frc.robot.wmlib2.util.FieldConstants
import frc.robot.wmlib2.util.GeomUtil
import java.util.function.Supplier
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.withSign

class SwerveJoystick(
        private val swerve: Swerve,
        private val xInput: Supplier<Double>,
        private val yInput: Supplier<Double>,
        private val rInput: Supplier<Double>
): Command(){

    val deadband = 0.1

    val linearSpeedLimit = 1.0
    val angularSpeedLimit = 25.0

    val maxLinearSpeedMetersPerSec = 0.0
    val minExtensionMaxAngularVelocity = 0.0

    override fun execute(){

        // Get current inputs from controller
        val xCurrent = xInput.get()
        val yCurrent = yInput.get()
        val rCurrent = rInput.get()

        // Get direction and magnitude of linear axes (x & y) & rotation
        var linearMagnitude = hypot(xCurrent, yCurrent)
        val linearDirection = Rotation2d(xCurrent, yCurrent)
        var angularSpeed = rCurrent

        // Apply deadband
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband)
        angularSpeed = MathUtil.applyDeadband(angularSpeed, deadband)

        // Apply squaring
        linearMagnitude = (linearMagnitude * linearMagnitude).withSign(linearMagnitude)
        angularSpeed = (angularSpeed * angularSpeed).withSign(angularSpeed)

        // Apply speed limits
        linearMagnitude *= linearSpeedLimit
        angularSpeed *= angularSpeedLimit

        // Calculate new linear components
        val linearVelocity: Translation2d = Pose2d(Translation2d(), linearDirection).transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0)).translation

        // Convert speeds to m/s
        var speeds = ChassisSpeeds(
            linearVelocity.x * maxLinearSpeedMetersPerSec,
            linearVelocity.y * maxLinearSpeedMetersPerSec,
            angularSpeed * minExtensionMaxAngularVelocity
        )

        // Always convert from field relative
        var currentRotation = swerve.getEstimatedRotation()
        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red){ // If there is not an option for the Optional<>! it will go to Blue
            currentRotation = currentRotation.plus(Rotation2d(Math.PI))
        }

        // Apply field relative
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                currentRotation
        )

        // Get current drive translation from vision, flip if need be
        var driveTranslation = AllianceFlipUtil.apply(swerve.getEstimatedPose().translation)

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
    }


}