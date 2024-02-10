// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.RotationOverride;

public class AutonomousSwerveAutoAimCommand extends Command {
  private Swerve s_Swerve;
  /** Creates a new AutonomousSwerveAutoAimCommand. */
  public AutonomousSwerveAutoAimCommand(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.setRotationTargetOverride(RotationOverride.SPEAKER_AUTO_AIM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.setRotationTargetOverride(RotationOverride.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
