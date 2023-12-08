
package frc.robot.wmlib2.swerve

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.wmlib2.swerve.IO_ModuleBase
import frc.robot.wmlib2.swerve.IO_ModuleBase.ModuleInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax
import com.revrobotics.AbsoluteEncoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.Constants
import frc.robot.Constants.MK4SDS
import frc.robot.Constants.ModuleSettings

/* REMOVED FOR NOW -> NEED TO GET REAL DRIVEBASE DONE!
// Abstracted from IO_IntakeBase, contains the code to simulate the hardware.
class IO_ModuleSim(val settings: ModuleSettings = Constants.ModuleSettings.DEFAULT) : IO_ModuleBase{

        private val driveSim = FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025)
        private val turnSim = FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004)

        private var turnRelativePositionRad = 0.0
        private var turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI
        private var driveAppliedVolts = 0.0
        private var turnAppliedVolts = 0.0

        override fun updateInputs(inputs: ModuleInputs) {
            driveSim.update(Constants.loopPeriodSecs)
            turnSim.update(Constants.loopPeriodSecs)

            val angleDiffRad = turnSim.angularVelocityRadPerSec * Constants.loopPeriodSecs
            turnRelativePositionRad += angleDiffRad
            turnAbsolutePositionRad += angleDiffRad

            while(turnAbsolutePositionRad < 0){
                turnAbsolutePositionRad += 2.0 * Math.PI
            }

            while(turnAbsolutePositionRad > 2.0 * Math.PI){
                turnAbsolutePositionRad -= 2.0 * Math.PI
            }

            inputs.drivePositionRad += driveSim.angularVelocityRadPerSec * Constants.loopPeriodSecs
            inputs.driveVelocityRadPerSec = driveSim.angularVelocityRadPerSec
            inputs.driveAppliedVolts = driveAppliedVolts
            inputs.driveCurrentAmps = driveSim.currentDrawAmps
            inputs.driveTempCelcius = 0.0

            inputs.turnAbsolutePositionRad = turnAbsolutePositionRad
            inputs.turnPositionRad = turnRelativePositionRad
            inputs.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
            inputs.turnAppliedVolts = turnAppliedVolts
            inputs.turnCurrentAmps = turnSim.currentDrawAmps
            inputs.turnTempCelcius = 0.0
        }

        override fun setDriveVoltage(voltage: Double){
            driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0)
            driveSim.setInputVoltage(driveAppliedVolts)
        }

        override fun setTurnVoltage(voltage: Double){
            turnAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0)
            turnSim.setInputVoltage(turnAppliedVolts)
        }

    }


 */