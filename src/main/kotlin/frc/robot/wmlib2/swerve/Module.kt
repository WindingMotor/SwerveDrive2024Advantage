
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
import kotlin.math.cos



class Module(
    private val io: IO_ModuleBase,
    private val settings: ModuleSettings
){

    private val inputs = IO_ModuleBase.ModuleInputs()

    // Returns the new optimized state of the module while sending motor voltage commands.
    fun runWithState(newState: SwerveModuleState): SwerveModuleState{

        // Optimize the desired module state based on the current module angle.
        val optimizedState = SwerveModuleState.optimize(newState, getModuleAngle())
    
        // Set both PIDs references, (Velocity & Position), to tell the module's SparkMaxes where to go
        io.setPIDReferences(optimizedState.speedMetersPerSecond, optimizedState.angle.radians)

        return optimizedState
    }

    fun periodic(){
        // Update the inputs.
        io.updateInputs(inputs)

        // Process inputs and send to logger.
        Logger.processInputs(settings.moduleName, inputs)
    }

    // Returns the current turn angle of the module. 
    fun getModuleAngle(): Rotation2d = Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad))

    // Returns the current drive position of the module in meters. 
    fun getModulePositionMeters(): Double = inputs.drivePositionRad * Constants.MK4SDS.WHEEL_DIAMETER

    // Returns the current drive velocity of the module in meters per second. 
    fun getVelocityMetersPerSec(): Double = inputs.driveVelocityRadPerSec * Constants.MK4SDS.WHEEL_DIAMETER

    // Returns the module position (turn angle and drive position).
    fun getModulePosition(): SwerveModulePosition = SwerveModulePosition(getModulePositionMeters(), getModuleAngle())

    // Returns the module state (turn angle and drive velocity).
    fun getModuleState(): SwerveModuleState = SwerveModuleState(getVelocityMetersPerSec(), getModuleAngle())

    fun stop(){io.stop()}

}

