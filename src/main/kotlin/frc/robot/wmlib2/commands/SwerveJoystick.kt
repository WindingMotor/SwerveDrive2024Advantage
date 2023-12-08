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
import frc.robot.wmlib2.util.AllianceFlipUtil
import frc.robot.wmlib2.util.GeomUtil
import java.util.function.Supplier
import kotlin.math.hypot
import kotlin.math.withSign

class SwerveJoystick(
        private val xInput: Supplier<Double>,
        private val yInput: Supplier<Double>,
        private val rInput: Supplier<Double>,
        private val isFieldOriented: Supplier<Boolean>, // Supplier can allow us to auto switch to non field oriented during a failure
        private val swerve: Swerve
): Command(){

    init{
        addRequirements(swerve)
    }

    val isFieldRelative = true

    override fun execute(){

        // Get current inputs from controller
        val xCurrent = xInput.get()
        val yCurrent = yInput.get()
        val rCurrent = rInput.get()

        // Create and send the newSpeeds using the inputs
        val newSpeeds = if(isFieldOriented.get()){ // Field oriented, requires gyro
            ChassisSpeeds.fromFieldRelativeSpeeds(xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY ,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY,
            swerve.gyroInputs.yawPosition)
        }else{ // Non-field oriented
            ChassisSpeeds(xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY ,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY)
        }

        swerve.runWithSpeeds(newSpeeds)

    }

    override fun end(interrupted: Boolean){
        swerve.stop()
    }

}