// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShintakeConstants;

public class Shintake extends SubsystemBase {
  
  private TalonFX m_flywheelLeft;
  private TalonFX m_flywheelRight;

  private TalonFXConfiguration flywheelConfigs;

  private VelocityVoltage velReq;

  private CANSparkMax m_intake;

  private DigitalInput intakeSensor;

  public Shintake() {
    m_flywheelLeft = new TalonFX(ShintakeConstants.leftShooterID);
    m_flywheelRight = new TalonFX(ShintakeConstants.rightShooterID);

    flywheelConfigs = new TalonFXConfiguration();

    m_intake = new CANSparkMax(ShintakeConstants.intakeID, MotorType.kBrushless);

    velReq = new VelocityVoltage(0).withSlot(0);

    intakeSensor = new DigitalInput(ShintakeConstants.irSensorPort);

    configMotors();
  }

  public void configMotors() {
    //Current Limiting  
    flywheelConfigs.CurrentLimits.SupplyCurrentLimit = 35;
    flywheelConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfigs.CurrentLimits.SupplyCurrentThreshold = 60;
    flywheelConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;

    flywheelConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    flywheelConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //PID Slot 0 (Velocity PID)
    flywheelConfigs.Slot0.kP = ShintakeConstants.shooterkP;
    flywheelConfigs.Slot0.kI = ShintakeConstants.shooterkI;
    flywheelConfigs.Slot0.kD = ShintakeConstants.shooterkD;

    flywheelConfigs.Slot0.kS = ShintakeConstants.shooterkS;
    flywheelConfigs.Slot0.kV = ShintakeConstants.shooterkV;
    flywheelConfigs.Slot0.kA = ShintakeConstants.shooterkA;

    //Feedback Configs
    // Gear ratio
    flywheelConfigs.Feedback.SensorToMechanismRatio = ShintakeConstants.flywheelGearing;

    //Add Configs
    m_flywheelLeft.getConfigurator().apply(flywheelConfigs);

    m_flywheelRight.getConfigurator().apply(flywheelConfigs);
    // Set right flywheel to opposite invert (CCW+ is "not inverted")
    m_flywheelRight.setInverted(
      flywheelConfigs.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive);

    //NEO 550 Intake Config
    m_intake.restoreFactoryDefaults();
    m_intake.setSmartCurrentLimit(20);
    m_intake.setInverted(false);
    m_intake.setIdleMode(IdleMode.kBrake);
    m_intake.enableVoltageCompensation(12);
  }

  //Methods

  public void stopAll() {
    stopIntake();
    stopShooter();
  }

  public void stopIntake() {
    m_intake.stopMotor();
  }

  public void stopShooter() {
    if (m_flywheelLeft.isAlive() && m_flywheelRight.isAlive()) {
      m_flywheelLeft.setControl(new NeutralOut());
      m_flywheelRight.setControl(new NeutralOut());
    }
  }

  public void setShooterVelocityRPS(double velocityLeft, double velocityRight) {
    if (m_flywheelLeft.isAlive() && m_flywheelRight.isAlive()){
      m_flywheelLeft.setControl(velReq.withVelocity(velocityLeft));
      m_flywheelRight.setControl(velReq.withVelocity(velocityRight));
    }
  }

  public void setShooterVelocityRPM(double velocityLeft, double velocityRight) {
    setShooterVelocityRPS(velocityLeft/60.0, velocityRight/60.0);
  }

  public void setIntakePercentSpeed(double speed) {
    m_intake.set(speed);
  }

  public double getLeftShooterVelocityRPS() {
    return m_flywheelLeft.getVelocity().getValue();
  }

  public double getRightShooterVelocityRPS() {
    return m_flywheelRight.getVelocity().getValue();
  }

  public double getLeftShooterVelocityRPM() {
    return getLeftShooterVelocityRPS()*60.0;
  }

  public double getRightShooterVelocityRPM() {
    return getRightShooterVelocityRPS()*60.0;
  }

  public double getIntakePercentSpeed(){
    return m_intake.get();
  }
  
  public boolean isNoteIntaked() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter Vel (rpm)", getLeftShooterVelocityRPM());
    SmartDashboard.putNumber("Right Shooter Vel (rpm)", getRightShooterVelocityRPM());
    SmartDashboard.putBoolean("IR Sensor", isNoteIntaked());

  }   
}