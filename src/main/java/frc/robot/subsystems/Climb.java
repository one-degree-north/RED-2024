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


package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX m_climb;
  private TalonFX m_climb2;
  private TalonFXConfiguration climbConfigs;
  private MotionMagicConfigs climbMotionMagicConfigs;
  private FeedbackConfigs climbFeedbackConfigs;
  private NeutralModeValue climbNeutralConfig = NeutralModeValue.Coast;
  private MotionMagicVoltage mmReq;
  private VelocityVoltage velReq;
  private DutyCycleEncoder climbEncoder;
  private DutyCycleEncoder climb2Encoder;
  private Boolean encodersAreReset;
  private Double climbOffset;
  private double count;
  
  public Climb() {
    m_climb = new TalonFX(0);
    m_climb2 = new TalonFX(0);
    climbConfigs = new TalonFXConfiguration();
    climbMotionMagicConfigs = new MotionMagicConfigs();
    climbFeedbackConfigs = new FeedbackConfigs();
    mmReq = new MotionMagicVoltage(0);
    velReq = new VelocityVoltage(0);
    climbEncoder = new DutyCycleEncoder(0);
    climb2Encoder = new DutyCycleEncoder(0);
    climbOffset = 0.0;

  }

  public void configMotors() {
    //Current Limiting  
    climbConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    climbConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfigs.CurrentLimits.SupplyCurrentThreshold = 60;
    climbConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
    climbConfigs.MotorOutput.NeutralMode = climbNeutralConfig;

    //PID Slot 0 (Motion Magic Position)
    climbConfigs.Slot0.kP = 0.0;
    climbConfigs.Slot0.kI = 0.0;
    climbConfigs.Slot0.kD = 0.0;

    climbConfigs.Slot0.kG = 0.0;
    climbConfigs.Slot0.kV = 0.0;
    climbConfigs.Slot0.kA = 0.0;

    climbConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    climbMotionMagicConfigs.MotionMagicCruiseVelocity = 0;
    climbMotionMagicConfigs.MotionMagicAcceleration = 0;

    //PID Slot 1 (Velocity)
    climbConfigs.Slot1.kP = 0.0;
    climbConfigs.Slot1.kI = 0.0;
    climbConfigs.Slot1.kD = 0.0;

    climbConfigs.Slot1.kG = 0.0;
    climbConfigs.Slot1.kV = 0.0;
    climbConfigs.Slot1.kA = 0.0;
    climbConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    //Feedback Configs
    climbFeedbackConfigs.SensorToMechanismRatio = 5;

    //Add Configs
    m_climb.getConfigurator().apply(climbConfigs);
    m_climb.getConfigurator().apply(climbConfigs.Feedback);
    m_climb.getConfigurator().apply(climbConfigs.MotorOutput);

    m_climb2.getConfigurator().apply(climbConfigs);
    m_climb2.getConfigurator().apply(climbConfigs.Feedback);
    m_climb2.getConfigurator().apply(climbConfigs.MotorOutput);

  }

  //Methods
  public void resetMotorToAbsolute(){
    double offsetClimbPos = climbEncoder.getAbsolutePosition() - climbOffset;
    double offsetClimb2Pos = climb2Encoder.getAbsolutePosition() - climbOffset;
    if (climbEncoder.isConnected() && climb2Encoder.isConnected()) { 
      m_climb.setPosition(offsetClimbPos);
      m_climb2.setPosition(offsetClimb2Pos);
      encodersAreReset = true;
    }
    else {
      encodersAreReset = false;
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

  public void setPositionLeft(double position) {
    mmReq.Position = position;
    setControl(m_climb, mmReq.withSlot(0));
  }

  public void setPositionRight(double position) {
    mmReq.Position = position;
    setControl(m_climb2, mmReq.withSlot(0));
  }

  public StatusSignal<Double> getClimbPos(){
    return m_climb.getPosition();
  }

  public StatusSignal<Double> getClimb2Pos(){
    return m_climb2.getPosition();
  }

  public double getAbsEncoderPos(){
    return climbEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {

    

    if (encodersAreReset && count <= 0) {
      resetMotorToAbsolute();
    } else {
      count += 1;
      count %= 5;
    }
  }
  
  //Set Climb Positions Later
  public static enum ClimbPosition {
    LEFT,
    CENTER,
    RIGHT;

    public double leftMotorPos() {
      switch (this) {
        case LEFT:
          return 0;
        case CENTER:
          return 0;
        case RIGHT:
          return 0;
        default:
          return 0;
      }
    }

    public double rightMotorPos() {
      switch (this) {
        case LEFT:
          return 0;
        case CENTER:
          return 0;
        case RIGHT:
          return 0;
        default:
          return 0;
      }
    }
  }
}
