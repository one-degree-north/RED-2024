// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatarmConstants;

public class Elevatarm extends SubsystemBase {
  /** Creates a new Elevatarm. */

  public TalonFX m_armLeader;
  public TalonFX m_armFollower;
  private TalonFX m_elevator;

  private DutyCycleEncoder m_armEncoder;
  private DutyCycleEncoder m_elevatorEncoder;

  private TalonFXConfiguration armConfig = new TalonFXConfiguration();

  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  private boolean isArmEncoderReset = false;
  private boolean isElevatorEncoderReset = false;

  private MotionMagicVoltage armMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut armDutyCycle = new DutyCycleOut(0);
  private double m_armMotionMagicSetpoint = 0;
  private double m_armDutyCycleOutput = 0;
  
  private MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private DutyCycleOut elevatorDutyCycle = new DutyCycleOut(0);
  private double m_elevatorMotionMagicSetpoint = 0;
  private double m_elevatorDutyCycleOutput = 0;

  private ControlState control = ControlState.OPEN_LOOP;

  private MechanismLigament2d elevatarmMech2d;

  private DigitalInput m_lockButton;

  private boolean isBraked = true;
  
  public Elevatarm() {
    m_armLeader = new TalonFX(ElevatarmConstants.armLeaderID, "*");
    m_armFollower = new TalonFX(ElevatarmConstants.armFollowerID, "*");
    // Set follower
    m_armFollower.setControl(new Follower(m_armLeader.getDeviceID(), true));
    
    m_armEncoder = new DutyCycleEncoder(ElevatarmConstants.armEncoderPort);
    
    m_lockButton = new DigitalInput(ElevatarmConstants.elevatarmLockSwitchPort);

    m_elevator = new TalonFX(ElevatarmConstants.elevatorID, "*");
    m_elevatorEncoder = new DutyCycleEncoder(ElevatarmConstants.elevatorEncoderPort);

    configMotors();

    Mechanism2d canvas = new Mechanism2d(4, 4);
    MechanismRoot2d root = canvas.getRoot("Pivot", 
      ElevatarmConstants.positionOfPivotRelativeToOrigin.getX(), 
      ElevatarmConstants.positionOfPivotRelativeToOrigin.getY());
    
    elevatarmMech2d = root.append(
      new MechanismLigament2d("Elevatarm", 
        m_elevator.getPosition().getValue()
        /ElevatarmConstants.elevatorMechanismRotationsToMetersRatio 
        + ElevatarmConstants.minDistanceOfShintakeRelativeToPivot, 
        Units.rotationsToDegrees(m_armLeader.getPosition().getValue())
      ));

    SmartDashboard.putData("Elevatarm", canvas);
  }

  private void configMotors() {
    // Arm config

    armConfig.Slot0.kP = ElevatarmConstants.armkP;
    armConfig.Slot0.kI = ElevatarmConstants.armkI;
    armConfig.Slot0.kD = ElevatarmConstants.armkD;
    armConfig.Slot0.kG = 0; // kG is set directly in control request
    armConfig.Slot0.kS = ElevatarmConstants.armkS;
    armConfig.Slot0.kV = ElevatarmConstants.armkV;
    armConfig.Slot0.kA = ElevatarmConstants.armkA;

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

    elevatorConfig.Slot0.kP = ElevatarmConstants.elevatorkP;
    elevatorConfig.Slot0.kI = ElevatarmConstants.elevatorkI;
    elevatorConfig.Slot0.kD = ElevatarmConstants.elevatorkD;
    elevatorConfig.Slot0.kG = 0; // kG is set directly in control request
    elevatorConfig.Slot0.kS = ElevatarmConstants.elevatorkS;
    elevatorConfig.Slot0.kV = ElevatarmConstants.elevatorkV;
    elevatorConfig.Slot0.kA = ElevatarmConstants.elevatorkA;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatarmConstants.elevatorCruiseVelocity
    * ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = ElevatarmConstants.elevatorAcceleration
    * ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;
    
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    elevatorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    elevatorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.Feedback.SensorToMechanismRatio = 
      ElevatarmConstants.elevatorIntegratedSensorToAbsoluteSensorRatio;

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatarmConstants.elevatorForwardSoftLimit
    * ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatarmConstants.elevatorReverseSoftLimit
    * ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;

    m_elevator.getConfigurator().apply(elevatorConfig);

    resetToAbsolute();
  }

  private void resetToAbsolute() {
    // TODO: Double check encoder ratios with design
    if (m_armEncoder.isConnected()) {
      double absolutePosition = 
        getArmAbsoluteEncoderAngle() 
        - ElevatarmConstants.armAbsoluteEncoderAngleOffset;
      /* The arm is configured so that on-board units is mechanism rotations */
      isArmEncoderReset = m_armLeader.setPosition(absolutePosition).isOK();
    }

    if (m_elevatorEncoder.isConnected()) {
      double absolutePosition =
        getElevatorAbsoluteEncoderDistance()
        - ElevatarmConstants.elevatorAbsoluteEncoderDistanceOffset;
      /* For the elevator, it is configured such that on-board units is mechanism meters.
       * This is safe to use as "reasonable" meter values lie within 
       * getPosition and getVelocity method return ranges
       */
      isElevatorEncoderReset = m_elevator.setPosition(absolutePosition
      * ElevatarmConstants.elevatorMechanismRotationsToMetersRatio).isOK();
    }
  }

  /* Rotations */
  private double getArmAbsoluteEncoderAngle() {
    return -m_armEncoder.getAbsolutePosition();
  }

  /* Meters */
  private double getElevatorAbsoluteEncoderDistance() {
    return -m_elevatorEncoder.getAbsolutePosition()
      / ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;
  }

  public Rotation2d getArmRotation2d() {
    return Rotation2d.fromRotations(m_armLeader.getPosition().getValue());
  }

  public double getElevatorMeters() {
    // TODO: Figure out conversion
    return m_elevator.getPosition().getValue()
    / ElevatarmConstants.elevatorMechanismRotationsToMetersRatio;
  }

  private void setControlArm(ControlRequest req) {
    if (m_armLeader.isAlive()
    && m_armFollower.isAlive()
    && isArmEncoderReset) {
      m_armLeader.setControl(req);
    }
  }

  private void setControlElevator(ControlRequest req) {
    if (m_elevator.isAlive() && 
    isElevatorEncoderReset) {
      m_elevator.setControl(req);
    }
  }

  /* In rotations */
  public void setArmPosition(double position) {
    control = ControlState.CLOSED_LOOP;
    m_armMotionMagicSetpoint = position;
  }

  public void setArmPercent(double percent) {
    control = ControlState.OPEN_LOOP;
    m_armDutyCycleOutput = percent;
    
  }

  public void setElevatorPosition(double position) {
    control = ControlState.CLOSED_LOOP;
    m_elevatorMotionMagicSetpoint = position;
  }

  public void setElevatorPercent(double percent) {
    control = ControlState.OPEN_LOOP;
    m_elevatorDutyCycleOutput = percent;
  }

  public boolean isArmEncoderReset() {
    return isArmEncoderReset;
  }

  public boolean isElevatorEncoderReset() {
    return isElevatorEncoderReset;
  }

  public double[] recalculateFeedForward() {
    // This assumes that arm feedforward is tuned with elevator at maximum extension
    double updatedArmkG = 
      ElevatarmConstants.armkG 
      * (Math.cos(getArmRotation2d().getRadians())) // Scale with arm angle
      * (ElevatarmConstants.minDistanceOfShintakeRelativeToPivot + getElevatorMeters())
        /(ElevatarmConstants.maxDistanceOfShintakeRelativeToPivot); // Scale with elevator extension
      // Max kG is when arm is horizontal and elevator is fully extended

    double updatedElevatorkG = 
      ElevatarmConstants.elevatorkG
      * Math.sin(getArmRotation2d().getRadians()); 
      // Scale with arm angle 
      // (sin because initial measurement is taken with upright arm)
      // Max kG is when arm is upright, min kG is when arm is horizontal
    SmartDashboard.putNumber("Arm kG", updatedArmkG);
    SmartDashboard.putNumber("Elevator kG", updatedElevatorkG);

    return new double[]{updatedArmkG, updatedElevatorkG};

  }

  public void setCoast() {
    if (isBraked){
      m_armLeader.setNeutralMode(NeutralModeValue.Coast);
      m_armFollower.setNeutralMode(NeutralModeValue.Coast);
      m_elevator.setNeutralMode(NeutralModeValue.Coast);
      isBraked = false;
    }
  }

  public void setBrake() {
    if (!isBraked) {
      m_armLeader.setNeutralMode(NeutralModeValue.Brake);
      m_armFollower.setNeutralMode(NeutralModeValue.Brake);
      m_elevator.setNeutralMode(NeutralModeValue.Brake);
      isBraked = true;
    }
  }
  

  @Override
  public void periodic() {
    switch (control) {
      case OPEN_LOOP:
        setControlArm(armDutyCycle.withOutput(m_armDutyCycleOutput));
        setControlElevator(elevatorDutyCycle.withOutput(m_elevatorDutyCycleOutput));
        break;
      case CLOSED_LOOP:
        setControlArm(armMotionMagic
          .withPosition(
            MathUtil.clamp(
              m_armMotionMagicSetpoint, 
              ElevatarmConstants.armReverseSoftLimit,
              ElevatarmConstants.armForwardSoftLimit)
          )
          .withFeedForward(recalculateFeedForward()[0])
        );

        setControlElevator(elevatorMotionMagic
          .withPosition(
            MathUtil.clamp(
              m_elevatorMotionMagicSetpoint*ElevatarmConstants.elevatorMechanismRotationsToMetersRatio, 
              ElevatarmConstants.elevatorReverseSoftLimit*ElevatarmConstants.elevatorMechanismRotationsToMetersRatio,
              ElevatarmConstants.elevatorForwardSoftLimit*ElevatarmConstants.elevatorMechanismRotationsToMetersRatio)
          )
          .withFeedForward(recalculateFeedForward()[1])
        );
        break;
    }

    if (DriverStation.isDisabled()) {
      if (m_lockButton.get()) {
        setBrake();
      } else if (!m_lockButton.get()) {
        setCoast();
      }
    }

    SmartDashboard.putBoolean("Brake mode", m_lockButton.get());

    elevatarmMech2d.setLength(
      getElevatorMeters()
      + ElevatarmConstants.minDistanceOfShintakeRelativeToPivot
    );
    
    elevatarmMech2d.setAngle(
      Units.rotationsToDegrees(getArmRotation2d().getRotations())
    );

    recalculateFeedForward();

    SmartDashboard.putNumber("Arm Pos (rot)", getArmRotation2d().getRotations());
    SmartDashboard.putNumber("Arm Through Bore Pos (rot)", getArmAbsoluteEncoderAngle());
    
    SmartDashboard.putNumber("Elevator Pos (m)", getElevatorMeters());
    SmartDashboard.putNumber("Elevator Through Bore Pos (m)", getElevatorAbsoluteEncoderDistance());

  }

  private enum ControlState {
    OPEN_LOOP, CLOSED_LOOP;
  }
}
