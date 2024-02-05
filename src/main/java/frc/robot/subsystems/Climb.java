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

    climbConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.climbCruiseVelocity;
    climbConfigs.MotionMagic.MotionMagicAcceleration = ClimbConstants.climbAcceleration;

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
    climbConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.climbForwardSoftLimit;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climbConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.climbReverseSoftLimit;

    //Feedback Configs
    climbConfigs.Feedback.SensorToMechanismRatio = ClimbConstants.climbIntegratedSensorToAbsoluteSensorRatio * ClimbConstants.climbMechanismRotationsToMeters;

    //Add Configs
    m_climbLeft.getConfigurator().apply(climbConfigs);
    m_climbRight.getConfigurator().apply(climbConfigs);

    // Make climb2 opposite of climb
    m_climbRight.setInverted(climbConfigs.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive);

    resetMotorsToAbsolute();
  }

  //Methods
  public void resetMotorsToAbsolute(){
    double offsetClimbPos = getLeftAbsoluteEncoderDistance() - ClimbConstants.leftClimbAbsoluteEncoderOffset;
    double offsetClimb2Pos = getRightAbsoluteEncoderDistance() - ClimbConstants.rightClimbAbsoluteEncoderOffset;
    if (m_climbLeftEncoder.isConnected() && m_climbRightEncoder.isConnected()) { 
      // only successful if no error
      boolean c1 = m_climbLeft.setPosition(offsetClimbPos).isOK();
      boolean c2 = m_climbRight.setPosition(offsetClimb2Pos).isOK();
      isClimbEncodersReset = c1 && c2;
    }
    
  }

  public void setControlLeft(ControlRequest req) {
    if (m_climbLeft.isAlive()
    && isClimbEncodersReset) {
      m_climbLeft.setControl(req);
    }
  }

  public void setControlRight(ControlRequest req) {
    if (m_climbRight.isAlive()
    && isClimbEncodersReset) {
      m_climbRight.setControl(req);
    }
  }

  public void setVelocityLeft(double velocity) {
    setControlLeft(climbVelocity.withVelocity(velocity));
  }

  public void setVelocityRight(double velocity) {
    setControlRight(climbVelocity.withVelocity(velocity));
  }

  public double getVelocityLeft() {
    return m_climbLeft.getVelocity().getValue();
  }

  public double getVelocityRight() {
    return m_climbRight.getVelocity().getValue();
  }

  public void setPositionLeft(double position) {
    setControlLeft(climbMotionMagic.withPosition(
      MathUtil.clamp(position, 
      ClimbConstants.climbReverseSoftLimit, 
      ClimbConstants.climbForwardSoftLimit)));
  }

  public void setPositionRight(double position) {
    setControlRight(climbMotionMagic.withPosition(
      MathUtil.clamp(position, 
      ClimbConstants.climbReverseSoftLimit, 
      ClimbConstants.climbForwardSoftLimit)
    ));
  }

  public void zeroVelocity() {
    setVelocityLeft(0);
    setVelocityRight(0);
  }

  public double getPositionLeft(){
    return m_climbLeft.getPosition().getValue();
  }

  public double getPositionRight(){
    return m_climbRight.getPosition().getValue();
  }

  public double getLeftAbsoluteEncoderDistance() {
    return m_climbLeftEncoder.getAbsolutePosition()
    /ClimbConstants.climbMechanismRotationsToMeters;
  }

  public double getRightAbsoluteEncoderDistance() {
    return m_climbRightEncoder.getAbsolutePosition()
    /ClimbConstants.climbMechanismRotationsToMeters;
  }

  public void enablePneumaticBreak() {
    m_leftPneumaticBreak.set(Value.kForward);
    m_rightPneumaticBreak.set(Value.kForward);
  }

  public void disablePneumaticBreak() {
    m_leftPneumaticBreak.set(Value.kReverse);
    m_rightPneumaticBreak.set(Value.kReverse);
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
