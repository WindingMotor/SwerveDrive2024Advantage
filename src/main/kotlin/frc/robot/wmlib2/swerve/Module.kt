
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import frc.robot.wmlib2.swerve.IO_ModuleBase.ModuleInputs
import com.revrobotics.CANSparkMax.IdleMode
import frc.robot.Constants.ModuleSettings
import frc.robot.Constants.MK4SDS
import frc.robot.Constants


class Module(
    private val io: IO_ModuleBase,
    private val settings: ModuleSettings
){

    private val inputs = IO_ModuleBase.ModuleInputs()

    private val driveFeedforward = SimpleMotorFeedforward(0.18868, 0.12825)
    private val driveFeedpack = PIDController(0.1, 0.0, 0.0, Constants.loopPeriodSecs)

    private val turnFeedback = PIDController(10.0, 0.0, 0.0, Constants.loopPeriodSecs)
    
    val wheelRadiusMeters = Constants.MK4SDS.WHEEL_DIAMETER

    // Returns the new optimized state of the module while sending motor voltage commands.
    fun runSetpoint(state: SwerveModuleState): SwerveModuleState{

        // Optimize the desired module state based on the current angle.
        val optimizedState = SwerveModuleState.optimize(state, getAngle())
    
        // Set the turn (rotation) voltage using a feedback controller.
        io.setTurnVoltage(turnFeedback.calculate(getAngle().radians, optimizedState.angle.radians))
    
        // Adjust the speed based on the turn feedback (possibly to account for strafing).
        optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.positionError)
    
        // Calculate the desired velocity in radians per second.
        val velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadiusMeters
    
        // Set the drive (linear motion) voltage using a feedforward controller.
        io.setDriveVoltage(driveFeedforward.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec))

        return optimizedState
    }

    fun periodic(){
        // Update the inputs.
        io.updateInputs(inputs)

        // Process inputs and send to logger.
        Logger.processInputs(settings.moduleName, inputs)
    }

    // Returns the current turn angle of the module. 
    fun getAngle(): Rotation2d = Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad))

    // Returns the current drive position of the module in meters. 
    fun getPositionMeters(): Double = inputs.drivePositionRad * wheelRadiusMeters

    // Returns the current drive velocity of the module in meters per second. 
    fun getVelocityMetersPerSec(): Double = inputs.driveVelocityRadPerSec * wheelRadiusMeters

    // Returns the module position (turn angle and drive position).
    fun getPosition(): SwerveModulePosition = SwerveModulePosition(getPositionMeters(), getAngle())

    // Returns the module state (turn angle and drive velocity).
    fun getState(): SwerveModuleState = SwerveModuleState(getVelocityMetersPerSec(), getAngle())

    // Returns the drive velocity in radians/sec.
    fun getCharacterizationVelocity(): Double = inputs.driveVelocityRadPerSec

    fun stop(){io.stop()}

}