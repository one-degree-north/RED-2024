// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatarmcommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevatarm;

public class ElevatarmTestCommand extends Command {
  private Elevatarm s_Elevatarm;
  private DoubleSupplier m_angleSupplier;
  private double m_elevatorExtension;

  /** Creates a new ArmManualControlCommand. */
public ElevatarmTestCommand(DoubleSupplier angleSupplier, double elevatorExtension, Elevatarm elevatarm) {
    this.s_Elevatarm = elevatarm;
    this.m_angleSupplier = angleSupplier;
    this.m_elevatorExtension = elevatorExtension;
    addRequirements(s_Elevatarm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevatarm.setArmPosition(m_angleSupplier.getAsDouble());
    s_Elevatarm.setElevatorPosition(m_elevatorExtension);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
