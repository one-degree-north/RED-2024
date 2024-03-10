// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbManualControlCommand extends Command {
  private double m_percent;
  private ClimbToMoveManual m_climbToMoveManual;
  private Climb s_Climb;
  private boolean m_closedLoop;

  /** Creates a new ClimbManualControlCommand. */
  public ClimbManualControlCommand(double percent, ClimbToMoveManual climbToMoveManual, Climb climb, boolean closedLoop) {
    m_percent = percent;
    m_climbToMoveManual = climbToMoveManual;
    s_Climb = climb;
    m_closedLoop = closedLoop;
    addRequirements(s_Climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_climbToMoveManual) {
      case LEFT:
        s_Climb.setDutyCycleLeft(m_percent);
        break;
      case RIGHT:
        s_Climb.setDutyCycleRight(m_percent);
        break;
      case BOTH:
        s_Climb.setDutyCycleLeft(m_percent);
        s_Climb.setDutyCycleRight(m_percent);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_closedLoop) {
      s_Climb.setSetpointPositionLeft(s_Climb.getPositionLeft());
      s_Climb.setSetpointPositionRight(s_Climb.getPositionRight());
    } else {
      s_Climb.setDutyCycleLeft(0);
      s_Climb.setDutyCycleRight(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum ClimbToMoveManual {
    LEFT, RIGHT, BOTH;
  }
}
