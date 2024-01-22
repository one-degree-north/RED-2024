// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Swerve;

public class AutoScore extends Command {
  /** Creates a new AutoScore. */

  private Command m_selectedCommand;
  private Supplier<AutoScorePosition> m_selectedPositionSupplier;
  private Pose2d m_selectedPosition;
  private Swerve m_swerve;
  private AutoScorePosition m_lastSelectedPosition;

  public AutoScore(Supplier<AutoScorePosition> selectedPositionSupplier, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_selectedPositionSupplier = selectedPositionSupplier;
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_selectedPosition = m_selectedPositionSupplier.get().getPose();
    m_lastSelectedPosition = m_selectedPositionSupplier.get();
    m_selectedCommand = m_swerve.goToPose(m_selectedPosition, 0, 0, true);
    m_selectedCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_selectedPositionSupplier.get() != m_lastSelectedPosition) {
      m_selectedPosition = m_selectedPositionSupplier.get().getPose();
      m_selectedCommand = m_swerve.goToPose(m_selectedPosition, 0, 0, true);
    }
    m_lastSelectedPosition = m_selectedPositionSupplier.get();
    
    m_selectedCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

  public static enum AutoScorePosition {
    LEFT, CENTER, RIGHT;
    private Pose2d getPose() {
      switch (this) {
        case LEFT:
          return new Pose2d(2.83, 7.03, new Rotation2d(0));
        case CENTER:
          return new Pose2d(2.89, 5.57, new Rotation2d(0));
        case RIGHT:
          return new Pose2d(2.91, 4.15, new Rotation2d(0));
        default:
          return new Pose2d(0, 0, new Rotation2d(0));
      }
    }
  }
}
