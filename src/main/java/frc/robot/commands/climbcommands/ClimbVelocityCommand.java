// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climb;

public class ClimbVelocityCommand extends Command {
  private Climb s_Climb;
  private double m_velocity;
  private ClimbToMove m_climbToMove;
  private Command m_commandToRun;

  /** Creates a new ClimbVelocityCommand. */
  public ClimbVelocityCommand(double velocityMPS, ClimbToMove climbToMove, Climb climb) {
    this.s_Climb = climb;
    addRequirements(s_Climb);

    this.m_velocity = velocityMPS;
    this.m_climbToMove = climbToMove;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(m_climbToMove) {
      case LEFT:
        m_commandToRun = new InstantCommand(() -> s_Climb.setVelocityLeft(m_velocity))
          .alongWith(new WaitUntilCommand(() -> false));
      case RIGHT:
        m_commandToRun = new InstantCommand(() -> s_Climb.setVelocityRight(m_velocity))
          .alongWith(new WaitUntilCommand(() -> false));
      case BOTH:
        m_commandToRun = new InstantCommand(() -> s_Climb.setVelocityLeft(m_velocity))
          .alongWith(new InstantCommand(() -> s_Climb.setVelocityRight(m_velocity)))
          .alongWith(new WaitUntilCommand(() -> false));
      default:
        m_commandToRun = new InstantCommand(() -> s_Climb.setVelocityLeft(m_velocity))
          .alongWith(new InstantCommand(() -> s_Climb.setVelocityRight(m_velocity)))
          .alongWith(new WaitUntilCommand(() -> false));
    }
    m_commandToRun.finallyDo(() -> s_Climb.zeroVelocity());

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

  public enum ClimbToMove {
    LEFT, RIGHT, BOTH;
  }
}
