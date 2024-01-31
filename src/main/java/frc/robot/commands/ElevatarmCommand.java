// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatarmConstants;
import frc.robot.subsystems.Elevatarm;

public class ElevatarmCommand extends Command {
  private Elevatarm s_Elevatarm;
  private double m_pivotAngle;
  private double m_extensionMeters;
  private Command m_commandToRun;

  /** Creates a new ElevatarmCommand. */
  public ElevatarmCommand(
    Rotation2d pivotAngle, 
    double extensionMeters, 
    Elevatarm elevatarm) {
      
    this.s_Elevatarm = elevatarm;
    this.m_pivotAngle = pivotAngle.getRotations();
    this.m_extensionMeters = extensionMeters;
    addRequirements(s_Elevatarm);
  }

  private boolean isElevatorAtSetpoint(double setpoint) {
    //3 cm
    return Math.abs(s_Elevatarm.getElevatorMeters()-setpoint) < 0.03;
  }

  private boolean isArmAtSetpoint(double setpoint) {
    // 0.01 rotations
    return Math.abs(s_Elevatarm.getArmRotation2d().getRotations()-setpoint) < 0.01;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_pivotAngle <= ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff) {
      /* Whenever the elevatarm is pivoting into danger area, clamp the elevator setpoint and always move elevator before pivoting arm. 
       * This prevents any end effector collision with the top of the drivebase
      */
      double elevatorClampedSetpoint = MathUtil.clamp(m_extensionMeters, 
              ElevatarmConstants.elevatorLowToGroundMinRetraction, ElevatarmConstants.elevatorForwardSoftLimit);

      m_commandToRun = 
            new InstantCommand(() -> s_Elevatarm.setElevatorPosition(elevatorClampedSetpoint))
              .alongWith(new WaitUntilCommand(() -> {return this.isElevatorAtSetpoint(elevatorClampedSetpoint);}))
            .andThen(
              new InstantCommand(() -> s_Elevatarm.setArmPosition(m_pivotAngle))
              .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle);}))
            )
      ;
    } else if (s_Elevatarm.getArmRotation2d().getRotations() <= ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff) {
      /* When the elevatarm is not pivoting within danger area BUT it is currently in danger area, pivot arm before moving elevator. 
       * This prevents any end effector collision with the front of the drivebase
      */
      m_commandToRun = 
        new InstantCommand(() -> s_Elevatarm.setArmPosition(m_pivotAngle))
              .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle);}))
            .andThen(
              new InstantCommand(() -> s_Elevatarm.setElevatorPosition(m_extensionMeters))
              .alongWith(new WaitUntilCommand(() -> {return this.isElevatorAtSetpoint(m_extensionMeters);}))
            )
      ;
    } else {
      /* Otherwise, pivot arm and extend elevator at the same time. */
      m_commandToRun = 
        new InstantCommand(() -> s_Elevatarm.setArmPosition(m_pivotAngle))
        .alongWith(new InstantCommand(() -> s_Elevatarm.setElevatorPosition(m_extensionMeters)))
        .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle) && this.isElevatorAtSetpoint(m_extensionMeters);}))
      ;
    }

    m_commandToRun.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_commandToRun.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_commandToRun.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }
}
