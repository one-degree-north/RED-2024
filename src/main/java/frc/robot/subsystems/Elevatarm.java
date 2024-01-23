// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatarmConstants;

public class Elevatarm extends SubsystemBase {
  /** Creates a new Elevatarm. */

  private TalonFX m_armLeader;
  private TalonFX m_armFollower;
  private DutyCycleEncoder m_armEncoder;

  private TalonFX m_elevator;
  private DutyCycleEncoder m_elevatorEncoder;
  private DigitalInput m_elevatorLimitSwitchMin;
  private DigitalInput m_elevatorLimitSwitchMax;

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private Slot0Configs armPIDConfigs = new Slot0Configs();

  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private Slot0Configs elevatorPIDConfigs = new Slot0Configs();

  private boolean isArmEncoderReset = false;
  private boolean isElevatorEncoderReset = false;
  private int count = 0;

  //TODO: create setpoints for arm motion magic
  private MotionMagicVoltage armMotionMagic = new MotionMagicVoltage(0);
  private DutyCycleOut armDutyCycle = new DutyCycleOut(0);
  
  private MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(0);
  private DutyCycleOut elevatorDutyCycle = new DutyCycleOut(0);
  
  public Elevatarm() {
    m_armLeader = new TalonFX(ElevatarmConstants.armLeaderID);
    m_armFollower = new TalonFX(ElevatarmConstants.armFollowerID);
    m_armEncoder = new DutyCycleEncoder(ElevatarmConstants.armEncoderPort);

    m_armFollower.setControl(new Follower(ElevatarmConstants.armLeaderID, true));

    m_elevator = new TalonFX(ElevatarmConstants.elevatorID);
    m_elevatorEncoder = new DutyCycleEncoder(ElevatarmConstants.elevatorEncoderPort);
    m_elevatorLimitSwitchMin = new DigitalInput(ElevatarmConstants.elevatorLimitMinPort);
    m_elevatorLimitSwitchMax = new DigitalInput(ElevatarmConstants.elevatorLimitMaxPort);

    configMotors();
  }

  private void configMotors() {
    // Arm config

    armPIDConfigs.kP = ElevatarmConstants.armkP;
    armPIDConfigs.kI = ElevatarmConstants.armkI;
    armPIDConfigs.kD = ElevatarmConstants.armkD;
    armPIDConfigs.kG = ElevatarmConstants.armkG;
    armPIDConfigs.kS = ElevatarmConstants.armkS;
    armPIDConfigs.kV = ElevatarmConstants.armkV;
    armPIDConfigs.kA = ElevatarmConstants.armkA;
    armPIDConfigs.GravityType = GravityTypeValue.Arm_Cosine;

    armConfig.Slot0 = armPIDConfigs;

    armConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatarmConstants.armCruiseVelocity;
    armConfig.MotionMagic.MotionMagicAcceleration = ElevatarmConstants.armAcceleration;
    
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 35;
    armConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    armConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.Feedback.SensorToMechanismRatio = ElevatarmConstants.armGearRatio;

    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatarmConstants.armForwardSoftLimit;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatarmConstants.armReverseSoftLimit;


    m_armLeader.getConfigurator().apply(armConfig);

    // Elevator config

    elevatorPIDConfigs.kP = ElevatarmConstants.elevatorkP;
    elevatorPIDConfigs.kI = ElevatarmConstants.elevatorkI;
    elevatorPIDConfigs.kD = ElevatarmConstants.elevatorkD;
    elevatorPIDConfigs.kG = ElevatarmConstants.elevatorkG;
    elevatorPIDConfigs.kS = ElevatarmConstants.elevatorkS;
    elevatorPIDConfigs.kV = ElevatarmConstants.elevatorkV;
    elevatorPIDConfigs.kA = ElevatarmConstants.elevatorkA;
    elevatorPIDConfigs.GravityType = GravityTypeValue.Elevator_Static;

    elevatorConfig.Slot0 = elevatorPIDConfigs;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatarmConstants.elevatorCruiseVelocity;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = ElevatarmConstants.elevatorAcceleration;
    
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    elevatorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    elevatorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatarmConstants.elevatorGearRatio;

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatarmConstants.elevatorForwardSoftLimit;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatarmConstants.elevatorReverseSoftLimit;

    m_elevator.getConfigurator().apply(elevatorConfig);

    resetToAbsolute();
  }

  private void resetToAbsolute() {
    if (m_armEncoder.isConnected()) {
      double absolutePosition = m_armEncoder.getAbsolutePosition() - ElevatarmConstants.armAbsoluteEncoderOffset;
      m_armLeader.setPosition(absolutePosition);
      isArmEncoderReset = true;
    }

    // TODO: figure out elevator conversions
    if (m_elevatorEncoder.isConnected()) {
      double absolutePosition = m_elevatorEncoder.getAbsolutePosition() - ElevatarmConstants.elevatorAbsoluteEncoderOffset;
      m_elevator.setPosition(absolutePosition);
      isElevatorEncoderReset = true;
    }
  }

  private void setControlArm(ControlRequest req) {
    if (isArmEncoderReset) {
      m_armLeader.setControl(req);
    }
  }

  private void setControlElevator(ControlRequest req) {
    if (isLimitSwitchTripped()) {
      m_elevator.setControl(new NeutralOut());
    }
    else if (isElevatorEncoderReset) {
      m_elevator.setControl(req);
    }
  }

  public void setArmPosition(double position) {
    setControlArm(armMotionMagic.withPosition(position));
  }

  public void setArmPercent(double percent) {
    setControlArm(armDutyCycle.withOutput(percent));
  }

  public void setElevatorPosition(double position) {
    setControlElevator(elevatorMotionMagic.withPosition(position));
  }

  public void setElevatorPercent(double percent) {
    setControlElevator(elevatorDutyCycle.withOutput(percent));
  }

  public boolean isLimitSwitchTripped() {
    return (m_elevatorLimitSwitchMin.get() && m_elevator.getVelocity().getValue() < 0) || 
    (m_elevatorLimitSwitchMax.get() && m_elevator.getVelocity().getValue() > 0);
  }

  public boolean isArmEncoderReset() {
    return isArmEncoderReset;
  }

  public boolean isElevatorEncoderReset() {
    return isElevatorEncoderReset;
  }
  

  @Override
  public void periodic() {
    // Keep trying to reset to absolute position
    count %=5;
    count += 1;
    if ((!isArmEncoderReset || !isElevatorEncoderReset) && count <=0) 
      resetToAbsolute();

    if (isLimitSwitchTripped()) {
      m_elevator.setControl(new NeutralOut());
    }
    
    
    // This method will be called once per scheduler run
  }
}
