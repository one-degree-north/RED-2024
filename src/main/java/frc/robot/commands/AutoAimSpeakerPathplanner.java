// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
/* This command should be used with a ParallelRaceCommand */

public class AutoAimSpeakerPathplanner extends Command {
  Swerve s_Swerve;
  /** Creates a new AutoAimAutonomousSwerve. */
  public AutoAimSpeakerPathplanner(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.enableSpeakerAutoAim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.disableSpeakerAutoAim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
