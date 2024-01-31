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

  private Compressor m_compressor;

  private TalonFXConfiguration climbConfigs;

  private MotionMagicVoltage mmReq;
  private VelocityVoltage velReq;

  private DutyCycleEncoder climbLeftEncoder;
  private DutyCycleEncoder climbRightEncoder;

  private boolean isEncodersReset = false;
  private double climbLeftOffset;
  private double climbRightOffset;
  private double count = 0;
  
  public Climb() {
    m_climbLeft = new TalonFX(ClimbConstants.leftClimbID);
    m_climbRight = new TalonFX(ClimbConstants.rightClimbID);

    m_leftPneumaticBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.leftPneumaticBreakPort1, ClimbConstants.leftPneumaticBreakPort2);
    m_leftPneumaticBreak = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.rightPneumaticBreakPort1, ClimbConstants.rightPneumaticBreakPort2);

    m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    climbConfigs = new TalonFXConfiguration();

    mmReq = new MotionMagicVoltage(0).withSlot(0);
    velReq = new VelocityVoltage(0).withSlot(1);

    climbLeftEncoder = new DutyCycleEncoder(ClimbConstants.leftEncoderPort);
    climbRightEncoder = new DutyCycleEncoder(ClimbConstants.rightEncoderPort);

    climbLeftOffset = ClimbConstants.leftAbsoluteEncoderOffset;
    climbRightOffset = ClimbConstants.rightAbsoluteEncoderOffset;

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
    m_climbLeft.getConfigurator().apply(climbConfigs);
    m_climbRight.getConfigurator().apply(climbConfigs);

    // Make climb2 opposite of climb
    m_climbRight.setInverted(climbConfigs.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive);

    resetMotorsToAbsolute();
  }

  //Methods
  public void resetMotorsToAbsolute(){
    double offsetClimbPos = getLeftAbsoluteEncoderDistance() - climbLeftOffset;
    double offsetClimb2Pos = getRightAbsoluteEncoderDistance() - climbRightOffset;
    if (climbLeftEncoder.isConnected() && climbRightEncoder.isConnected()) { 
      // only successful if no error
      boolean c1 = m_climbLeft.setPosition(offsetClimbPos).isOK();
      boolean c2 = m_climbRight.setPosition(offsetClimb2Pos).isOK();
      isEncodersReset = c1 && c2;
    }
    
  }

  public void setControl(TalonFX motor, ControlRequest req) {
    if (isEncodersReset) {
      motor.setControl(req);
    }
  }

  public void setVelocityLeft(double velocity) {
    setControl(m_climbLeft, 
    velReq.withVelocity(velocity));
  }

  public void setVelocityRight(double velocity) {
    setControl(m_climbRight, 
    velReq.withVelocity(velocity));
  }

  public double getVelocityLeft() {
    return m_climbLeft.getVelocity().getValue();
  }

  public double getVelocityRight() {
    return m_climbRight.getVelocity().getValue();
  }

  public void setPositionLeft(double position) {
    setControl(m_climbLeft, 
    mmReq.withPosition(
      MathUtil.clamp(position, 
      ClimbConstants.reverseSoftLimit, 
      ClimbConstants.forwardSoftLimit)));
  }

  public void setPositionRight(double position) {
    setControl(m_climbRight, 
    mmReq.withPosition(
      MathUtil.clamp(position, 
      ClimbConstants.reverseSoftLimit, 
      ClimbConstants.forwardSoftLimit)
    ));
  }

  public void stopVelocity() {
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
    return climbLeftEncoder.getAbsolutePosition()
    /ClimbConstants.absoluteSensorToMotorRatio
    /ClimbConstants.encoderRotationsPerDistance;
  }

  public double getRightAbsoluteEncoderDistance() {
    return climbRightEncoder.getAbsolutePosition()
    /ClimbConstants.absoluteSensorToMotorRatio
    /ClimbConstants.encoderRotationsPerDistance;
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
    // too lazy to fix aaden weird code (it works so whatever)
    if (!isEncodersReset && count <= 0) {
      resetMotorsToAbsolute();
    } else if (!isEncodersReset) {
      count += 1;
      count %= 5;
    }

    SmartDashboard.putNumber("Left Climb Pos (m)", getPositionLeft());
    SmartDashboard.putNumber("Right Climb Pos (m)", getPositionRight());

    SmartDashboard.putNumber("Left Through Bore Pos (m)", getLeftAbsoluteEncoderDistance());
    SmartDashboard.putNumber("Right Through Bore Pos (m)", getRightAbsoluteEncoderDistance());

    SmartDashboard.putNumber("Pressure (PSI)", m_compressor.getPressure());
  }
  
}
