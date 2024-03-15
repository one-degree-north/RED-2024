// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.subsystems.Elevatarm;
import frc.robot.subsystems.Swerve;

public class AutonomousElevatarmAutoAimCommand extends Command {
  private Elevatarm s_Elevatarm;
  private Swerve s_Swerve;

  /** Creates a new ElevatarmAutoAimCommand. */
  public AutonomousElevatarmAutoAimCommand(Elevatarm elevatarm, Swerve swerve) {
    this.s_Elevatarm = elevatarm;
    this.s_Swerve = swerve;
    // do not make swerve a requirement as it is just used to find setpoint
    addRequirements(s_Elevatarm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Elevatarm.setElevatorPosition(MechanismSetpointConstants.elevatorGroundIntakePosition/2);
    s_Elevatarm.setArmPosition(s_Swerve.getShotData().clampedArmAngle());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
