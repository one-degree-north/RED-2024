// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatarmcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevatarm;

public class ArmManualControlCommand extends Command {
  private Elevatarm s_Elevatarm;
  private double m_percent;
  private boolean m_closedLoop;

  /** Creates a new ArmManualControlCommand. */
  public ArmManualControlCommand(double percent, Elevatarm elevatarm, boolean closedLoop) {
    this.s_Elevatarm = elevatarm;
    this.m_percent = percent;
    this.m_closedLoop = closedLoop;
    addRequirements(s_Elevatarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevatarm.setArmPercent(m_percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_closedLoop)
      s_Elevatarm.setArmPosition(s_Elevatarm.getArmRotation2d().getRotations());
    else
      s_Elevatarm.setArmPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
