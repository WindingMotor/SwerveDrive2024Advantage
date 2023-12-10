
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
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.Constants
import frc.robot.Constants.MK4SDS
import frc.robot.Constants.ModuleSettings


// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
class IO_ModuleReal(val settings: ModuleSettings = Constants.ModuleSettings.DEFAULT) : IO_ModuleBase{

    // Create the drive motor
    private val driveMotor = CANSparkMax(settings.driveID, MotorType.kBrushless).apply{
        setIdleMode(IdleMode.kBrake)
        setSmartCurrentLimit(45);
        setInverted(false);
        burnFlash();
    }

    // Create the drive encoder
    private val driveEncoder = driveMotor.encoder.apply{ 
        positionConversionFactor =  Constants.MK4SDS.DRIVE_ROT_2_METER
        velocityConversionFactor = Constants.MK4SDS.DRIVE_RPM_2_MPS
    }

    // Create the turn motor
    private val turnMotor = CANSparkMax(settings.turnID, MotorType.kBrushless).apply{
        setIdleMode(IdleMode.kBrake)
        setSmartCurrentLimit(25);
        setInverted(false);
        burnFlash();
    }

    // Create the turn encoder
    private val turnEncoder = turnMotor.encoder.apply{ 
        positionConversionFactor =  Constants.MK4SDS.TURN_ROT_2_RAD
        velocityConversionFactor = Constants.MK4SDS.TURN_RPM_2_RADPS
    }

    // Create the absolute encoder
    private val turnAbsoluteEncoder = DutyCycleEncoder(settings.absoluteEncoderID).apply{
         setDutyCycleRange(1.0/4096.0, 4095.0/4096.0)
    }

    // Create module PIDs
    private val turnPID = turnMotor.getPIDController().apply{
        setP(Constants.MK4SDS.TURN_MODULE_PID_P);
        setI(Constants.MK4SDS.TURN_MODULE_PID_I);
        setD(Constants.MK4SDS.TURN_MODULE_PID_D);
        setOutputRange(-1.0, 1.0);
        setPositionPIDWrappingEnabled(true);
        setPositionPIDWrappingMinInput(0.0);
        setPositionPIDWrappingMaxInput(2*Math.PI);
        setFeedbackDevice(turnMotor.encoder);
    }

    private val drivePID = driveMotor.getPIDController().apply{
        setP(Constants.MK4SDS.DRIVE_MODULE_PID_P);
        setI(Constants.MK4SDS.DRIVE_MODULE_PID_I);
        setD(Constants.MK4SDS.DRIVE_MODULE_PID_D);
        setOutputRange(-1.0, 1.0);
        setFeedbackDevice(driveMotor.encoder);
        //setFF(1.0 / ( Constants.MK4SDS.FREE_MOTOR_SPEED_RPS * Constants.MK4SDS.WHEEL_DIAMETER * Math.PI));
    }

    // Update inputs with current values
    override fun updateInputs(inputs: ModuleInputs){
        inputs.drivePositionRad = driveEncoder.position
        inputs.driveVelocityRadPerSec = driveEncoder.velocity
        inputs.driveAppliedPercentage = driveMotor.appliedOutput
        inputs.driveCurrentAmps = driveMotor.outputCurrent
        inputs.driveTempCelcius = driveMotor.motorTemperature

        // Get absolute position -> Convert to radians -> Wrap from -pi to pi -> Subtract offset -> Wrap again
        inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
            Rotation2d(MathUtil.angleModulus(Math.toRadians(turnAbsoluteEncoder.absolutePosition))).minus(settings.absoluteEncoderOffset).radians
        )

        inputs.turnPositionRad = turnEncoder.position
        inputs.turnVelocityRadPerSec = turnEncoder.velocity
        inputs.driveAppliedPercentage = turnMotor.appliedOutput
        inputs.turnCurrentAmps = turnMotor.outputCurrent
        inputs.turnTempCelcius = turnMotor.motorTemperature
    }

    override fun setDriveOutput(percent: Double){
        driveMotor.set(percent)
    }

    override fun setTurnOutput(percent: Double){
        turnMotor.set(percent)
    }

    override fun stop(){
        driveMotor.set(0.0)
        turnMotor.set(0.0)
    }

    override fun setPIDReferences(driveReference: Double, turnReference: Double){
        drivePID.setReference(driveReference, CANSparkMax.ControlType.kVelocity);
        turnPID.setReference(turnReference, CANSparkMax.ControlType.kPosition);
    }


}


