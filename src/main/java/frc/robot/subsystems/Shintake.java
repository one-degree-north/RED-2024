// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shintake extends SubsystemBase {
  
  private TalonFX m_flywheel;
  private TalonFX m_flywheel2;
  private TalonFXConfiguration flywheelConfigs;
  private FeedbackConfigs flywheelFeedbackConfigs;
  private NeutralModeValue flywheelNeutralConfig = NeutralModeValue.Coast;
  private VelocityVoltage velReq;
  private CANSparkMax m_intake;
  private boolean ringIntaked;
  private DigitalInput intakeSensor;

  public Shintake() {
    m_flywheel = new TalonFX(0);
    m_flywheel2 = new TalonFX(0);
    flywheelConfigs = new TalonFXConfiguration();
    flywheelFeedbackConfigs = new FeedbackConfigs();
    m_intake = new CANSparkMax(0, MotorType.kBrushless);
    velReq = new VelocityVoltage(0);
    intakeSensor = new DigitalInput(0);
    
  }

  public void configMotors() {
    //Current Limiting  
    flywheelConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    flywheelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfigs.CurrentLimits.SupplyCurrentThreshold = 60;
    flywheelConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
    flywheelConfigs.MotorOutput.NeutralMode = flywheelNeutralConfig;
    m_flywheel.setInverted(false);
    m_flywheel2.setInverted(false);

    //PID Slot 0 (Velocity PID)
    flywheelConfigs.Slot0.kP = 0.0;
    flywheelConfigs.Slot0.kI = 0.0;
    flywheelConfigs.Slot0.kD = 0.0;

    flywheelConfigs.Slot0.kS = 0.0;
    flywheelConfigs.Slot0.kV = 0.0;
    flywheelConfigs.Slot0.kA = 0.0;

    //Feedback Configs
    flywheelFeedbackConfigs.SensorToMechanismRatio = 5.16;

    //Add Configs
    m_flywheel.getConfigurator().apply(flywheelConfigs);
    m_flywheel.getConfigurator().apply(flywheelConfigs.Feedback);
    m_flywheel.getConfigurator().apply(flywheelConfigs.MotorOutput);

    m_flywheel2.getConfigurator().apply(flywheelConfigs);
    m_flywheel2.getConfigurator().apply(flywheelConfigs.Feedback);
    m_flywheel2.getConfigurator().apply(flywheelConfigs.MotorOutput);

    //NEO 550 Config
    m_intake.restoreFactoryDefaults();
    m_intake.setSmartCurrentLimit(20);
    m_intake.setInverted(false);

  }

  //Methods

  public void setVelocity(double velocity) {
    velReq.Velocity = velocity;
    if(ringIntaked) {
      m_flywheel.setControl(velReq.withSlot(0));
      m_flywheel2.setControl(velReq.withSlot(0));
    }
  }

  public void intakeRing(double speed) {
    if (ringIntaked = false) {
      m_intake.set(speed);
    }
  }

  public StatusSignal<Double> getFlywheelRot(){
    return m_flywheel.getVelocity();
  }

  public StatusSignal<Double> getFlywheel2Rot(){
    return m_flywheel2.getVelocity();
  }

  public double getIntakeSpeed(){
    return m_intake.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakeSensor.get()) {
      ringIntaked = true;
    } else {
      ringIntaked = false;
    }
  }
}