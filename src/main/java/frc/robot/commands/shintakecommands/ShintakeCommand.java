// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintakecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.subsystems.Shintake;

public class ShintakeCommand extends Command {
  /** Creates a new ShintakeCommand. */
  private ShintakeMode m_mode;
  private Shintake s_Shintake;
  private Command m_commandToRun;
  private boolean m_stopWhenFinished;

  public ShintakeCommand(ShintakeMode mode, Shintake shintake, boolean stopWhenFinished) {
    this.s_Shintake = shintake;
    addRequirements(s_Shintake);

    this.m_mode = mode;
    this.m_stopWhenFinished = stopWhenFinished;
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
      
      case THROW_NOTE_TO_SPEAKER:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(ShintakeConstants.throwNoteRPM, ShintakeConstants.throwNoteRPM))
          .alongWith(
            new WaitCommand(ShintakeConstants.shooterTimeoutRampTimeSeconds)
            .raceWith(
              new WaitUntilCommand(() -> {
                return 
                Math.abs(s_Shintake.getLeftShooterVelocityRPM() - ShintakeConstants.throwNoteRPM)
                < MechanismSetpointConstants.flywheelVelocityAllowableError
                && 
                Math.abs(s_Shintake.getRightShooterVelocityRPM() - ShintakeConstants.throwNoteRPM)
                < MechanismSetpointConstants.flywheelVelocityAllowableError;
              }
              )
            )
          )
          .andThen(
              new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
              .alongWith(
                new WaitUntilCommand(() -> !s_Shintake.isNoteIntaked())
                .andThen(new WaitCommand(ShintakeConstants.shooterDelaySeconds))
              )
          )
        ;
        break;
      case SHOOT:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(ShintakeConstants.shooterLeftRPM, ShintakeConstants.shooterRightRPM))
          .alongWith(
            new WaitCommand(ShintakeConstants.shooterTimeoutRampTimeSeconds)
            .raceWith(
              new WaitUntilCommand(() -> {
                return 
                Math.abs(s_Shintake.getLeftShooterVelocityRPM() - ShintakeConstants.shooterLeftRPM)
                < MechanismSetpointConstants.flywheelVelocityAllowableError
                && 
                Math.abs(s_Shintake.getRightShooterVelocityRPM() - ShintakeConstants.shooterRightRPM)
                < MechanismSetpointConstants.flywheelVelocityAllowableError;
              }
              )
            )
          )
          .andThen(
              new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
              .alongWith(
                new WaitUntilCommand(() -> !s_Shintake.isNoteIntaked())
                .andThen(new WaitCommand(ShintakeConstants.shooterDelaySeconds))
              )
          )
        ;
        break;
      case JUSTFEEDTOSHOOT:
        m_commandToRun = 
          new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
          .alongWith(
            new WaitUntilCommand(() -> !s_Shintake.isNoteIntaked())
            .andThen(new WaitCommand(ShintakeConstants.shooterDelaySeconds))
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
    if (m_stopWhenFinished)
      s_Shintake.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }

  public enum ShintakeMode {
    FRONT_OUTTAKE, AMP_AND_TRAP, GROUND_INTAKE, SOURCE_INTAKE, SHOOT,
    JUSTFEEDTOSHOOT, THROW_NOTE_TO_SPEAKER;
  }
}
