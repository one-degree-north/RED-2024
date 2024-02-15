// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatarmcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevatarm;

public class ElevatorManualControlCommand extends Command {
  private Elevatarm s_Elevatarm;
  private double m_percent;

  /** Creates a new ElevatorManualControlCommand. */
  public ElevatorManualControlCommand(double percent, Elevatarm elevatarm) {
    this.s_Elevatarm = elevatarm;
    this.m_percent = percent;
    addRequirements(s_Elevatarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevatarm.setElevatorPercent(m_percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Elevatarm.setElevatorPosition(s_Elevatarm.getElevatorMeters());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}