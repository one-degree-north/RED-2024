// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* To Do:
 * 
 * Motion Magic Position (For going up), Velocity PID (For robot climb/down)
 * 
 * Use slot 0 and slot 1 for differnet gains
 * 
 * 3 Methods for height when going up (left, center, right),
 * for left and right the arms of climb have to be at different heights
 * 
 * Climb is left, Cimb2 is right
 */

 // This subsystem has been checked over

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX m_climb;
  private TalonFX m_climb2;

  private TalonFXConfiguration climbConfigs;

  private MotionMagicVoltage mmReq;
  private VelocityVoltage velReq;

  private DutyCycleEncoder climbEncoder;
  private DutyCycleEncoder climb2Encoder;

  private Boolean encodersAreReset = false;
  private double climbOffset;
  private double climb2Offset;
  private double count;
  
  public Climb() {
    m_climb = new TalonFX(ClimbConstants.leftClimbID);
    m_climb2 = new TalonFX(ClimbConstants.rightClimbID);

    climbConfigs = new TalonFXConfiguration();

    mmReq = new MotionMagicVoltage(0);
    velReq = new VelocityVoltage(0);

    climbEncoder = new DutyCycleEncoder(ClimbConstants.leftEncoderPort);
    climb2Encoder = new DutyCycleEncoder(ClimbConstants.rightEncoderPort);

    climbOffset = ClimbConstants.leftAbsoluteEncoderOffset;
    climb2Offset = ClimbConstants.rightAbsoluteEncoderOffset;


  }

  public void configMotors() {
    //Current Limiting  
    climbConfigs.CurrentLimits.SupplyCurrentLimit = 35;
    climbConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfigs.CurrentLimits.SupplyCurrentThreshold = 60;
    climbConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;

    climbConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //PID Slot 0 (Motion Magic Position)
    climbConfigs.Slot0.kP = ClimbConstants.positionkP;
    climbConfigs.Slot0.kI = ClimbConstants.positionkI;
    climbConfigs.Slot0.kD = ClimbConstants.positionkD;

    climbConfigs.Slot0.kG = ClimbConstants.positionkG;
    climbConfigs.Slot0.kS = ClimbConstants.positionkS;
    climbConfigs.Slot0.kV = ClimbConstants.positionkV;
    climbConfigs.Slot0.kA = ClimbConstants.positionkA;

    climbConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    climbConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.cruiseVelocity;
    climbConfigs.MotionMagic.MotionMagicAcceleration = ClimbConstants.acceleration;

    //PID Slot 1 (Velocity)
    climbConfigs.Slot1.kP = ClimbConstants.velocitykP;
    climbConfigs.Slot1.kI = ClimbConstants.velocitykI;
    climbConfigs.Slot1.kD = ClimbConstants.velocitykD;

    climbConfigs.Slot1.kG = ClimbConstants.velocitykG;
    climbConfigs.Slot1.kS = ClimbConstants.velocitykS;
    climbConfigs.Slot1.kV = ClimbConstants.velocitykV;
    climbConfigs.Slot1.kA = ClimbConstants.velocitykA;
    climbConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    //Software Limit Switches
    climbConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.forwardSoftLimit;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.reverseSoftLimit;

    //Feedback Configs
    climbConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.encoderRotationsPerDistance;

    //Add Configs
    m_climb.getConfigurator().apply(climbConfigs);
    m_climb2.getConfigurator().apply(climbConfigs);

    // Make climb2 opposite of climb
    m_climb2.setInverted(climbConfigs.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive);

    resetMotorsToAbsolute();
  }

  //Methods
  public void resetMotorsToAbsolute(){
    double offsetClimbPos = getLeftAbsoluteEncoderDistance() - climbOffset;
    double offsetClimb2Pos = getRightAbsoluteEncoderDistance() - climb2Offset;
    if (climbEncoder.isConnected() && climb2Encoder.isConnected()) { 
      // only successful if no error
      boolean c1 = m_climb.setPosition(offsetClimbPos).isOK();
      boolean c2 = m_climb2.setPosition(offsetClimb2Pos).isOK();
      encodersAreReset = c1 && c2;
    }
    
  }

  public void setControl(TalonFX motor, ControlRequest req) {
    if (encodersAreReset) {
      motor.setControl(req);
    }
  }

  public void setVelocityLeft(double velocity) {
    velReq.Velocity = velocity;
    setControl(m_climb, velReq.withSlot(1));
  }

  public void setVelocityRight(double velocity) {
    velReq.Velocity = velocity;
    setControl(m_climb2, velReq.withSlot(1));
  }

  public double getVelocityLeft() {
    return m_climb.getVelocity().getValue();
  }

  public double getVelocityRight() {
    return m_climb2.getVelocity().getValue();
  }

  public void setPositionLeft(double position) {
    mmReq.Position = position;
    setControl(m_climb, mmReq.withSlot(0));
  }

  public void setPositionRight(double position) {
    mmReq.Position = position;
    setControl(m_climb2, mmReq.withSlot(0));
  }

  public double getClimbLeftPos(){
    return m_climb.getPosition().getValue();
  }

  public double getClimbRightPos(){
    return m_climb2.getPosition().getValue();
  }

  public double getLeftAbsoluteEncoderDistance() {
    return climbEncoder.getAbsolutePosition()
    /ClimbConstants.absoluteSensorToMotorRatio
    /ClimbConstants.encoderRotationsPerDistance;
  }

  public double getRightAbsoluteEncoderDistance() {
    return climb2Encoder.getAbsolutePosition()
    /ClimbConstants.absoluteSensorToMotorRatio
    /ClimbConstants.encoderRotationsPerDistance;
  }

  @Override
  public void periodic() {
    // too lazy to fix aaden weird code (it works so whatever)
    if (!encodersAreReset && count <= 0) {
      resetMotorsToAbsolute();
    } else if (!encodersAreReset) {
      count += 1;
      count %= 5;
    }
  }
  
}
