// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.subsystems.Climb;

public class ClimbPositionCommand extends Command {
  private Climb s_Climb;
  private ClimbPosition m_position;
  private Command m_commandToRun;
  /** Creates a new ClimbCommand. */
  public ClimbPositionCommand(ClimbPosition position, Climb climb) {
    this.s_Climb = climb;
    addRequirements(s_Climb);

    this.m_position = position;
  }

  private boolean isLeftClimbAtSetpoint() {
    return Math.abs(s_Climb.getPositionLeft()-m_position.leftClimbSetpoint()) < MechanismSetpointConstants.climbAllowableError;
  }

  private boolean isRightClimbAtSetpoint() {
    return Math.abs(s_Climb.getPositionRight()-m_position.rightClimbSetpoint()) < MechanismSetpointConstants.climbAllowableError;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_commandToRun = new InstantCommand(() -> s_Climb.setPositionLeft(m_position.leftClimbSetpoint()))
      .alongWith(new InstantCommand(() -> s_Climb.setPositionRight(m_position.rightClimbSetpoint())))
      .alongWith(new WaitUntilCommand(() -> isLeftClimbAtSetpoint() && isRightClimbAtSetpoint()))
    ;

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

  public enum ClimbPosition {
    LEFTHIGH, MIDDLE, RIGHTHIGH;

    public double leftClimbSetpoint() {
      switch (this) {
        case LEFTHIGH:
          return MechanismSetpointConstants.climbHighPosition;
        case MIDDLE:
          return MechanismSetpointConstants.climbStandardPosition;
        case RIGHTHIGH:
          return MechanismSetpointConstants.climbLowPosition;
        default:
          return MechanismSetpointConstants.climbStandardPosition;
      }
    }

    public double rightClimbSetpoint() {
      switch (this) {
        case LEFTHIGH:
          return MechanismSetpointConstants.climbLowPosition;
        case MIDDLE:
          return MechanismSetpointConstants.climbStandardPosition;
        case RIGHTHIGH:
          return MechanismSetpointConstants.climbHighPosition;
        default:
          return MechanismSetpointConstants.climbStandardPosition;
      }
    }

  }
}
