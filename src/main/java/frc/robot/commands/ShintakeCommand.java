// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

public class ShintakeCommand extends Command {
  /** Creates a new ShintakeCommand. */
  private ShintakeMode m_mode;
  private Shintake s_Shintake;
  private Command m_commandToRun;

  public ShintakeCommand(ShintakeMode mode, Shintake shintake) {
    this.s_Shintake = shintake;
    addRequirements(s_Shintake);

    this.m_mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (m_mode) {
      case FRONT_OUTTAKE:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(-ShintakeConstants.outtakeRPM, -ShintakeConstants.outtakeRPM))
          .alongWith(new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(-ShintakeConstants.outtakePercentSpeed)))
          .alongWith(new WaitUntilCommand(() -> false))
        ;
        break;
      case AMP_AND_TRAP:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(ShintakeConstants.outtakeRPM, ShintakeConstants.outtakeRPM))
          .alongWith(new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.outtakePercentSpeed)))
          .alongWith(new WaitUntilCommand(() -> !s_Shintake.isNoteIntaked()).andThen(new WaitCommand(ShintakeConstants.ampAndTrapDelaySeconds)))
        ;
        break;
      case GROUND_INTAKE:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
          .alongWith(new WaitUntilCommand(() -> s_Shintake.isNoteIntaked()))
        ;
        break;
      case SOURCE_INTAKE:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(-ShintakeConstants.outtakePercentSpeed))
          .alongWith(new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(-ShintakeConstants.outtakeRPM, -ShintakeConstants.outtakeRPM)))
          .alongWith(new WaitUntilCommand(() -> s_Shintake.isNoteIntaked()).andThen(new WaitCommand(ShintakeConstants.sourceIntakeDelaySeconds)))
        ;
        break;
      case SHOOT:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(ShintakeConstants.shooterLeftRPM, ShintakeConstants.shooterRightRPM))
          .alongWith(new WaitCommand(ShintakeConstants.shooterRampTimeSeconds))
          .andThen(
              new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
              .alongWith(
                new WaitUntilCommand(() -> !s_Shintake.isNoteIntaked())
                .andThen(new WaitCommand(ShintakeConstants.shooterDelaySeconds))
              )
          )
        ;
        break;
      default:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
          .alongWith(new WaitUntilCommand(() -> s_Shintake.isNoteIntaked()))
        ;
        break;
    }
    m_commandToRun.finallyDo(() -> s_Shintake.stopAll());

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

  public enum ShintakeMode {
    FRONT_OUTTAKE, AMP_AND_TRAP, GROUND_INTAKE, SOURCE_INTAKE, SHOOT
  }
}
