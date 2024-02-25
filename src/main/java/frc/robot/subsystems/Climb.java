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
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private TalonFX m_climbLeft;
  private TalonFX m_climbRight;

  private DoubleSolenoid m_leftPneumaticBreak;
  private DoubleSolenoid m_rightPneumaticBreak;

  private DutyCycleEncoder m_climbLeftEncoder;
  private DutyCycleEncoder m_climbRightEncoder;

  private Compressor m_compressor;

  private TalonFXConfiguration climbConfigs = new TalonFXConfiguration();

  private MotionMagicVoltage climbMotionMagic = new MotionMagicVoltage(0).withSlot(0);;
  private VelocityVoltage climbVelocity = new VelocityVoltage(0).withSlot(1);;

  private boolean isClimbEncodersReset = false;
  
  public Climb() {
    m_climbLeft = new TalonFX(ClimbConstants.leftClimbID);
    m_climbRight = new TalonFX(ClimbConstants.rightClimbID);

    m_leftPneumaticBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.leftPneumaticBreakPort1, ClimbConstants.leftPneumaticBreakPort2);
    m_leftPneumaticBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.rightPneumaticBreakPort1, ClimbConstants.rightPneumaticBreakPort2);

    m_climbLeftEncoder = new DutyCycleEncoder(ClimbConstants.leftClimbEncoderPort);
    m_climbRightEncoder = new DutyCycleEncoder(ClimbConstants.rightClimbEncoderPort);

    m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    configMotors();
    enableCompressor();
  }

  public void enableCompressor() {
    m_compressor.enableAnalog(70, 120);
  }

  public void disableCompressor() {
    m_compressor.disable();
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
    climbConfigs.Slot0.kP = ClimbConstants.climbPositionkP;
    climbConfigs.Slot0.kI = ClimbConstants.climbPositionkI;
    climbConfigs.Slot0.kD = ClimbConstants.climbPositionkD;

    climbConfigs.Slot0.kG = ClimbConstants.climbPositionkG;
    climbConfigs.Slot0.kS = ClimbConstants.climbPositionkS;
    climbConfigs.Slot0.kV = ClimbConstants.climbPositionkV;
    climbConfigs.Slot0.kA = ClimbConstants.climbPositionkA;

    climbConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    climbConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.climbCruiseVelocity
      *ClimbConstants.climbMechanismRotationsToMetersRatio;
    climbConfigs.MotionMagic.MotionMagicAcceleration = ClimbConstants.climbAcceleration
      *ClimbConstants.climbMechanismRotationsToMetersRatio;

    //PID Slot 1 (Velocity)
    climbConfigs.Slot1.kP = ClimbConstants.climbVelocitykP;
    climbConfigs.Slot1.kI = ClimbConstants.climbVelocitykI;
    climbConfigs.Slot1.kD = ClimbConstants.climbVelocitykD;

    climbConfigs.Slot1.kG = ClimbConstants.climbVelocitykG;
    climbConfigs.Slot1.kS = ClimbConstants.climbVelocitykS;
    climbConfigs.Slot1.kV = ClimbConstants.climbVelocitykV;
    climbConfigs.Slot1.kA = ClimbConstants.climbVelocitykA;
    climbConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    //Software Limit Switches
    climbConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climbConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.climbForwardSoftLimit
      *ClimbConstants.climbMechanismRotationsToMetersRatio;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.climbReverseSoftLimit
      *ClimbConstants.climbMechanismRotationsToMetersRatio;

    //Feedback Configs
    climbConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.climbIntegratedSensorToAbsoluteSensorRatio;

    //Add Configs
    m_climbLeft.getConfigurator().apply(climbConfigs);
    m_climbRight.getConfigurator().apply(climbConfigs);

    // Make climb2 opposite of climb
    m_climbRight.setInverted(climbConfigs.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive);

    resetMotorsToAbsolute();
  }

  //Methods
  public void resetMotorsToAbsolute(){
    double offsetLeftClimbDistance = getLeftAbsoluteEncoderDistance() - ClimbConstants.leftClimbAbsoluteEncoderOffset;
    double offsetRightClimbDistance = getRightAbsoluteEncoderDistance() - ClimbConstants.rightClimbAbsoluteEncoderOffset;
    if (m_climbLeftEncoder.isConnected() && m_climbRightEncoder.isConnected()) { 
      // only successful if no error
      boolean c1 = setEncoderPositionLeft(offsetLeftClimbDistance);
      boolean c2 = setEncoderPositionRight(offsetRightClimbDistance);
      isClimbEncodersReset = c1 && c2;
    }
    
  }

  public boolean setControlLeft(ControlRequest req) {
    if (m_climbLeft.isAlive()
    && isClimbEncodersReset) {
      return m_climbLeft.setControl(req).isOK();
    }
    return false;
  }

  public boolean setControlRight(ControlRequest req) {
    if (m_climbRight.isAlive()
    && isClimbEncodersReset) {
      return m_climbRight.setControl(req).isOK();
    }
    return false;
  }

  public boolean areEncodersReset() {
    return isClimbEncodersReset;
  }

  // position in meters
  public boolean setEncoderPositionLeft(double meters) {
    return m_climbLeft.setPosition(meters
        *ClimbConstants.climbMechanismRotationsToMetersRatio).isOK();
  }

  // position in meters
  public boolean setEncoderPositionRight(double meters) {
    return m_climbRight.setPosition(meters
        *ClimbConstants.climbMechanismRotationsToMetersRatio).isOK();
  }

  /* In meters per second */
  public boolean setVelocityLeft(double velocity) {
    return setControlLeft(climbVelocity.withVelocity(velocity
    *ClimbConstants.climbMechanismRotationsToMetersRatio));
  }

  public boolean setVelocityRight(double velocity) {
    return setControlRight(climbVelocity.withVelocity(velocity
    *ClimbConstants.climbMechanismRotationsToMetersRatio));
  }

  public double getVelocityLeft() {
    return m_climbLeft.getVelocity().getValue()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public double getVelocityRight() {
    return m_climbRight.getVelocity().getValue()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public boolean setSetpointPositionLeft(double position) {
    return setControlLeft(climbMotionMagic.withPosition(
      MathUtil.clamp(
      position
      *ClimbConstants.climbMechanismRotationsToMetersRatio, 

      ClimbConstants.climbReverseSoftLimit
      *ClimbConstants.climbMechanismRotationsToMetersRatio, 

      ClimbConstants.climbForwardSoftLimit
      *ClimbConstants.climbMechanismRotationsToMetersRatio
      )));
  }

  public boolean setSetpointPositionRight(double position) {
    return setControlRight(climbMotionMagic.withPosition(
      MathUtil.clamp(
        position
        *ClimbConstants.climbMechanismRotationsToMetersRatio, 

        ClimbConstants.climbReverseSoftLimit
        *ClimbConstants.climbMechanismRotationsToMetersRatio, 

        ClimbConstants.climbForwardSoftLimit
        *ClimbConstants.climbMechanismRotationsToMetersRatio
      )
    ));
  }

  public void zeroVelocity() {
    setVelocityLeft(0);
    setVelocityRight(0);
  }

  public double getPositionLeft(){
    return m_climbLeft.getPosition().getValue()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public double getPositionRight(){
    return m_climbRight.getPosition().getValue()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public double getLeftAbsoluteEncoderDistance() {
    return m_climbLeftEncoder.getAbsolutePosition()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public double getRightAbsoluteEncoderDistance() {
    return m_climbRightEncoder.getAbsolutePosition()
    /ClimbConstants.climbMechanismRotationsToMetersRatio;
  }

  public void enablePneumaticBreak() {
    m_leftPneumaticBreak.set(Value.kForward);
    m_rightPneumaticBreak.set(Value.kForward);
  }

  public void disablePneumaticBreak() {
    m_leftPneumaticBreak.set(Value.kReverse);
    m_rightPneumaticBreak.set(Value.kReverse);
  }

  public void disableClimbMotors() {
    setControlLeft(new NeutralOut());
    setControlRight(new NeutralOut());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Climb Pos (m)", getPositionLeft());
    SmartDashboard.putNumber("Right Climb Pos (m)", getPositionRight());

    SmartDashboard.putNumber("Left Through Bore Pos (m)", getLeftAbsoluteEncoderDistance());
    SmartDashboard.putNumber("Right Through Bore Pos (m)", getRightAbsoluteEncoderDistance());

    SmartDashboard.putNumber("Pressure (PSI)", m_compressor.getPressure());
  }
  
}
