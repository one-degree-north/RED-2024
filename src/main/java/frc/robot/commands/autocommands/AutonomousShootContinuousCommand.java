// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.commands.elevatarmcommands.ElevatarmCommand;
import frc.robot.commands.shintakecommands.ShintakeCommand;
import frc.robot.commands.shintakecommands.ShintakeCommand.ShintakeMode;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevatarm;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Swerve;

public class AutonomousShootContinuousCommand extends Command {
  private Command m_commandToRun;
  private boolean m_ended;

  private Shintake s_Shintake;
  private Swerve s_Swerve;
  private Elevatarm s_Elevatarm;
  private Climb s_Climb;
  /** Creates a new AutonomousShootContinuousCommand. */
  public AutonomousShootContinuousCommand(Shintake shintake, Swerve swerve, Elevatarm elevatarm, Climb climb) {
    this.s_Shintake = shintake;
    this.s_Swerve = swerve;
    this.s_Elevatarm = elevatarm;
    this.s_Climb = climb;
    addRequirements(s_Shintake, s_Elevatarm);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shintake.setShooterVelocityRPM(ShintakeConstants.shooterLeftRPM, ShintakeConstants.shooterRightRPM);
    m_commandToRun = new SequentialCommandGroup(
      // intake and deploy intake only until note is intaked/only if note isnt intaked
      new ShintakeCommand(ShintakeMode.GROUND_INTAKE, s_Shintake, false).finallyDo(() -> s_Shintake.stopIntake())
      .alongWith(
        new ElevatarmCommand(
          MechanismSetpointConstants.armGroundIntakePosition, 
          MechanismSetpointConstants.elevatorGroundIntakePosition, 
          s_Elevatarm, s_Climb)
      )
      .onlyIf(() -> !s_Shintake.isNoteIntaked())
      
      ,

      // run auto aim commands together
      new AutonomousSwerveAutoAimCommand(s_Swerve)
      .alongWith(
      new AutonomousElevatarmAutoAimCommand(s_Elevatarm, s_Swerve)
      )
      // race the auto aim commands with the following sequence:
        // wait until arm + swerve are at setpoints (swerve must be behind x position cutoff, and slower than specified velocity)
        // after arm + swerve are at setpoints, feed the note into shooter
      .raceWith(new WaitUntilCommand(() -> {
            return Math.abs(
              MathUtil.angleModulus(
                s_Swerve.getShotData().goalHeading()
                .minus(s_Swerve.getPose().getRotation())
                .getRadians()
              )
            ) 
            < MechanismSetpointConstants.swerveRotationAllowableError
            &&
            Math.abs(
              s_Swerve.getShotData().clampedArmAngle()
              - s_Elevatarm.getArmRotation2d().getRotations()
            )
            < MechanismSetpointConstants.armAllowableError
            &&
            s_Swerve.getShotData().effectiveRobotToSpeakerDist()
            < MechanismSetpointConstants.distanceCutoffToAutoScore
            &&
            Math.abs(s_Swerve.getTranslationalSpeed())
            < MechanismSetpointConstants.allowableVelocityToAutoScore
            &&
            s_Swerve.getShotData().inYDistanceRange()
            && 
            Math.abs(s_Shintake.getLeftShooterVelocityRPM() - ShintakeConstants.shooterLeftRPM)
            < MechanismSetpointConstants.flywheelVelocityAllowableError
            && 
            Math.abs(s_Shintake.getRightShooterVelocityRPM() - ShintakeConstants.shooterRightRPM)
            < MechanismSetpointConstants.flywheelVelocityAllowableError
            ;
        })
        .andThen(
          new ShintakeCommand(ShintakeMode.JUSTFEEDTOSHOOT, s_Shintake, false).finallyDo(() -> s_Shintake.stopIntake())
        )
      )
    );
    // when this sequence ends, it repeats (will deploy intake again)
    m_ended = false;
    m_commandToRun.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ended) {
      m_ended = false;
      m_commandToRun.initialize();
    }
    m_commandToRun.execute();
    if (m_commandToRun.isFinished()) {
      // restart command
      m_commandToRun.end(false);
      m_ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Make sure we didn't already call end() (which would happen if the command finished in the
    // last call to our execute())
    if (!m_ended) {
      m_commandToRun.end(interrupted);
      m_ended = true;
    }
    if (interrupted) {
      s_Shintake.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
