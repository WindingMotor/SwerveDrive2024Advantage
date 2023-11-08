
package frc.robot.subsystems.intake

import org.littletonrobotics.junction.inputs.LoggableInputs
import org.littletonrobotics.junction.LogTable
import frc.robot.subsystems.intake.IO_IntakeBase
import frc.robot.subsystems.intake.IO_IntakeBase.IntakeInputs
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim

// Abstracted from IO_IntakeBase, contains the code to simulate the robot hardware
class IO_IntakeSim : IO_IntakeBase{

    // A simple intake arm simulation, values taken from here: https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/cubeintake/CubeIntakeIOSim.java
    private val armSim = SingleJointedArmSim(DCMotor.getNEO(1), 50.0, 0.5, 0.5, 0.0, Math.PI / 2, true, 0.0).apply{ 
        setState(VecBuilder.fill(Math.PI / 2.0, 0.0))
    }

    private var armAppliedVoltage = 0.0
    
    init{}
    // Update inputs with simulated values
    override fun updateInputs(inputs: IntakeInputs){

        // Update armSim every 0.02 seconds
        armSim.update(0.02)

        inputs.appliedVoltage = armAppliedVoltage
        inputs.motorTemperature = 0.2
        inputs.encoderVelocity = 0.3
    }

    // Set motor speed 
    override fun setMotorVoltage(voltage: Double){
        armAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0)
        armSim.setInputVoltage(armAppliedVoltage)
    }

    // Set motor idle mode
    override fun setMotorIdleMode(mode: IdleMode){}

}