// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathGenerationConstants;
import frc.robot.subsystems.Swerve;

public class AutoScorePathfind extends Command {
  /** Creates a new AutoScore. */

  private Command m_selectedCommand;
  private Supplier<AutoScorePosition> m_selectedPositionSupplier;
  private AutoScorePosition m_lastSelectedPosition;
  private boolean m_ended;


  private Swerve m_swerve;

  public AutoScorePathfind(Supplier<AutoScorePosition> selectedPositionSupplier, Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_selectedPositionSupplier = selectedPositionSupplier;
    m_swerve = swerve;
    m_ended = false;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ended = false;
    // m_lastSelectedPosition = m_selectedPositionSupplier.get();
    m_selectedCommand = m_swerve.goToPose(m_selectedPositionSupplier.get().getPose(), 0, 0, true);
    m_selectedCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ended) {
      m_ended = false;
      initialize();
    }
    m_selectedCommand.execute();

    if (m_lastSelectedPosition != m_selectedPositionSupplier.get()) {
      end(false);
      m_ended = true;
    }

    m_lastSelectedPosition = m_selectedPositionSupplier.get();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_ended) {
      m_selectedCommand.end(interrupted);
      m_ended = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Make this return false if it should "infinitely pathfind"
    return m_selectedCommand.isFinished();
  }

  public static enum AutoScorePosition {
    // TODO: add amp scoring pose, add actual scoring poses
    LEFT, CENTER, RIGHT;
    private Pose2d getPose() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        // Swap right with left on red alliance
        switch (this) {
          case LEFT:
            return PathGenerationConstants.rightSpeakerScoringPose;
          case CENTER:
            return PathGenerationConstants.middleSpeakerScoringPose;
          case RIGHT:
            return PathGenerationConstants.leftSpeakerScoringPose;
          default:
            return PathGenerationConstants.middleSpeakerScoringPose;
        }
      } else {
        // Regular orientation since standard is blue origin
        switch (this) {
          case LEFT:
            return PathGenerationConstants.leftSpeakerScoringPose;
          case CENTER:
            return PathGenerationConstants.middleSpeakerScoringPose;
          case RIGHT:
            return PathGenerationConstants.rightSpeakerScoringPose;
          default:
            return PathGenerationConstants.middleSpeakerScoringPose;
        }
      }
    }
  }
}
