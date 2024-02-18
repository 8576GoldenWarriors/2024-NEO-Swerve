// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;    //com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private PearadoxSparkMax driveMotor;
  private PearadoxSparkMax turnMotor;

  private int driveMotorId;
  private int turnMotorId;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;
  private Rotation2d lastAngle;

  //CANcoderConfiguration config = new CANcoderConfiguration();


  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, double magnetOffset) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      //CANcoderConfiguration config = new CANcoderConfiguration();
      //config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

      //absoluteEncoder.getConfigurator().apply(config);

      driveMotor = new PearadoxSparkMax(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      turnMotor = new PearadoxSparkMax(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      this.driveMotorId = driveMotorId;
      this.turnMotorId = turnMotorId;

      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      new WaitCommand(0.1);
      absoluteEncoder = new CANcoder(absoluteEncoderId);

      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.MagnetOffset = magnetOffset;

      config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

      absoluteEncoder.getConfigurator().apply(config);

      turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(0, 1);//-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getState().angle;
      //System.out.println("Motor ID" + turnMotorId + " Angle " + getState().angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setIdleMode(IdleMode.kBrake);
      //turnMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kBrake);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    System.out.println("Absolute Encoder: "+ absoluteEncoder.getAbsolutePosition());
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    //angle /= 360;
    angle-=absoluteEncoderOffset;
    return angle;
    // angle -= absoluteEncoderOffset;
    //angle *= (Math.PI / 180);
    // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    //System.out.println("Turn Enocder: "+turnEncoder.getPosition());
    driveEncoder.setPosition(0);

    System.out.print(turnMotorId + " ");
    
    //System.out.println(getAbsoluteEncoderAngle());

    //System.out.println("TurnMotor Conversion" + absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    //System.out.println(turnEncoder.getPosition());
    turnEncoder.setPosition((getAbsoluteEncoderAngle()));// / SwerveConstants.TURN_MOTOR_PCONVERSION);

    //SmartDashboard.putNumber("Turn encoder" + turnMotorId, getAbsoluteEncoderAngle());
    SmartDashboard.putString("Turn encoder" + turnMotorId, turnEncoder.toString());
    //System.out.println(turnEncoder.getPosition());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    //Logger.recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());
//    SmartDashboard.putNumber("Turn Encoder " + turnEncoder, absoluteEncoderOffset)
  }

  public void setRawState(SwerveModuleState desiredState){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    setRawAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    //Logger.recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());
    
  }

  public void setSpeed(SwerveModuleState desiredState){
    double speed =  desiredState.speedMetersPerSecond;
    if(speed>SwerveConstants.DRIVETRAIN_MAX_SPEED){
      speed = SwerveConstants.DRIVETRAIN_MAX_SPEED;
    }
    driveMotor.set(speed); //desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED    original parameter, changed for debugging purposes #KP 12/2

    // driveController.setReference(
    //   desiredState.speedMetersPerSecond,
    //   ControlType.kVelocity,
    //   0,
    //   feedforward.calculate(desiredState.speedMetersPerSecond));
  }

  public void setAngle(SwerveModuleState desiredState){
    // Rotation2d angle = desiredState.angle;

    //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; 
    //System.out.println(turnMotorId + " Angle " + angle);
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), (4 * (double) desiredState.angle.getRotations())));

    //System.out.println(desiredState.angle.getRotations());
    lastAngle = angle;
  }

  public void setRawAngle(SwerveModuleState desiredState){
    Rotation2d angle = desiredState.angle;

    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), (double) desiredState.angle.getRotations()));
    lastAngle = angle;
  }
  
  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
}