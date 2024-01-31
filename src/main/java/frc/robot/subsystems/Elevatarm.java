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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private MotionMagicVoltage armMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut armDutyCycle = new DutyCycleOut(0);
  
  private MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut elevatorDutyCycle = new DutyCycleOut(0);

  private MechanismLigament2d m_elevatarmMech;
  
  public Elevatarm() {
    m_armLeader = new TalonFX(ElevatarmConstants.armLeaderID);
    m_armFollower = new TalonFX(ElevatarmConstants.armFollowerID);
    m_armEncoder = new DutyCycleEncoder(ElevatarmConstants.armEncoderPort);

    m_elevator = new TalonFX(ElevatarmConstants.elevatorID);
    m_elevatorEncoder = new DutyCycleEncoder(ElevatarmConstants.elevatorEncoderPort);
    m_elevatorLimitSwitchMin = new DigitalInput(ElevatarmConstants.elevatorLimitMinPort);
    m_elevatorLimitSwitchMax = new DigitalInput(ElevatarmConstants.elevatorLimitMaxPort);

    configMotors();

    Mechanism2d canvas = new Mechanism2d(3, 3);
    MechanismRoot2d root = canvas.getRoot("Pivot", 
      ElevatarmConstants.pivotRelativeToOrigin.getX(), 
      ElevatarmConstants.pivotRelativeToOrigin.getY());
    
    m_elevatarmMech = root.append(
      new MechanismLigament2d("Elevatarm", 
        m_elevator.getPosition().getValue() + ElevatarmConstants.minRetractionEE, 
        Units.rotationsToDegrees(m_armLeader.getPosition().getValue())
      ));
    SmartDashboard.putData("Elevatarm", canvas);
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

    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatarmConstants.elevatorEncoderRotationsPerDistance;

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatarmConstants.elevatorForwardSoftLimit;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatarmConstants.elevatorReverseSoftLimit;

    m_elevator.getConfigurator().apply(elevatorConfig);

    // Set follower
    m_armFollower.setControl(new Follower(ElevatarmConstants.armLeaderID, true));

    resetToAbsolute();
  }

  private void resetToAbsolute() {
    // TODO: Double check encoder ratios with design
    if (m_armEncoder.isConnected()) {
      double absolutePosition = 
        getArmAbsoluteEncoderAngle() 
        - ElevatarmConstants.armAbsoluteEncoderOffset;
      isArmEncoderReset = m_armLeader.setPosition(absolutePosition).isOK();
    }

    if (m_elevatorEncoder.isConnected()) {
      double absolutePosition =
        getElevatorAbsoluteEncoderDistance()
        - ElevatarmConstants.elevatorAbsoluteEncoderOffset;
      isElevatorEncoderReset = m_elevator.setPosition(absolutePosition).isOK();
    }
  }

  private double getArmAbsoluteEncoderAngle() {
    return m_armEncoder.getAbsolutePosition();
  }

  private double getElevatorAbsoluteEncoderDistance() {
    return (m_elevatorEncoder.getAbsolutePosition()
      / ElevatarmConstants.elevatorAbsoluteSensorToMotorRatio) 
      / ElevatarmConstants.elevatorEncoderRotationsPerDistance;
  }

  public Rotation2d getArmRotation2d() {
    return Rotation2d.fromRotations(m_armLeader.getPosition().getValue());
  }

  public double getElevatorMeters() {
    // TODO: Figure out conversion
    return m_elevator.getPosition().getValue();
  }

  private void setControlArm(ControlRequest req) {
    if (isArmEncoderReset) {
      m_armLeader.setControl(req);
    }
  }

  private void setControlElevator(ControlRequest req) {
    if (isElevatorEncoderReset) {
      m_elevator.setControl(req);
    }
  }

  public void setArmPosition(double position) {
    setControlArm(armMotionMagic.withPosition(
      MathUtil.clamp(
        position, 
        ElevatarmConstants.armReverseSoftLimit,
        ElevatarmConstants.armForwardSoftLimit))
    );
  }

  public void setArmPercent(double percent) {
    setControlArm(armDutyCycle.withOutput(percent));
  }

  public void setElevatorPosition(double position) {
    setControlElevator(elevatorMotionMagic.withPosition(
        MathUtil.clamp(
        position, 
        ElevatarmConstants.elevatorReverseSoftLimit,
        ElevatarmConstants.elevatorForwardSoftLimit)
    )
      .withLimitForwardMotion(m_elevatorLimitSwitchMax.get())
      .withLimitReverseMotion(m_elevatorLimitSwitchMin.get())
    );
  }

  public void setElevatorPercent(double percent) {
    setControlElevator(elevatorDutyCycle.withOutput(percent)
      .withLimitForwardMotion(m_elevatorLimitSwitchMax.get())
      .withLimitReverseMotion(m_elevatorLimitSwitchMin.get())
    );
  }

  public boolean isArmEncoderReset() {
    return isArmEncoderReset;
  }

  public boolean isElevatorEncoderReset() {
    return isElevatorEncoderReset;
  }

  public void recalculateFeedForward() {
    // This assumes that arm feedforward is tuned with elevator at maximum extension
    armPIDConfigs.kG = 
      ElevatarmConstants.armkG*(ElevatarmConstants.minRetractionEE+getElevatorMeters())
        /(ElevatarmConstants.maxExtensionEE);
    elevatorPIDConfigs.kG = 
      ElevatarmConstants.elevatorkG*Math.sin(getArmRotation2d().getRadians());
    m_armLeader.getConfigurator().apply(armPIDConfigs);
    m_elevator.getConfigurator().apply(elevatorPIDConfigs);
  }

  public void setCoast() {
    m_armLeader.setNeutralMode(NeutralModeValue.Coast);
    m_armFollower.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrake() {
    m_armLeader.setNeutralMode(NeutralModeValue.Brake);
    m_armFollower.setNeutralMode(NeutralModeValue.Brake);
  }
  

  @Override
  public void periodic() {
    // Keep trying to reset to absolute position
    if ((!isArmEncoderReset || !isElevatorEncoderReset) && count <=0) {
      resetToAbsolute();
    }
    else {
      count %=5;
      count += 1;
    }

    m_elevatarmMech.setLength(
      m_elevator.getPosition().getValue() 
      + ElevatarmConstants.minRetractionEE
    );
    
    m_elevatarmMech.setAngle(
      Units.rotationsToDegrees(m_armLeader.getPosition().getValue())
    );

    recalculateFeedForward();

    SmartDashboard.putNumber("Arm Pos (rot)", getArmRotation2d().getRotations());
    SmartDashboard.putNumber("Arm Through Bore Pos (rot)", getArmAbsoluteEncoderAngle());
    
    SmartDashboard.putNumber("Elevator Pos (m)", getElevatorMeters());
    SmartDashboard.putNumber("Elevator Through Bore Pos (m)", getElevatorAbsoluteEncoderDistance());
  }
}
