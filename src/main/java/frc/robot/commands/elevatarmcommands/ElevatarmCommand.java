// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatarmcommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatarmConstants;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.commands.climbcommands.ClimbPositionCommand;
import frc.robot.commands.climbcommands.ClimbPositionCommand.ClimbPosition;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevatarm;

public class ElevatarmCommand extends Command {
  private Elevatarm s_Elevatarm;
  private Climb s_Climb;
  private double m_pivotAngle;
  private double m_extensionMeters;
  private Command m_commandToRun;

  /** Creates a new ElevatarmCommand. */
  public ElevatarmCommand(
    double pivotAngle, 
    double extensionMeters, 
    Elevatarm elevatarm,
    Climb climb) {
      
    this.s_Elevatarm = elevatarm;
    this.s_Climb = climb;
    this.m_pivotAngle = pivotAngle;
    this.m_extensionMeters = extensionMeters;
    addRequirements(s_Elevatarm);
  }

  private boolean isElevatorAtSetpoint(double setpoint) {
    //3 cm
    return Math.abs(s_Elevatarm.getElevatorMeters()-setpoint) < MechanismSetpointConstants.elevatorAllowableError;
  }

  private boolean isArmAtSetpoint(double setpoint) {
    // 0.01 rotations
    return Math.abs(s_Elevatarm.getArmRotation2d().getRotations()-setpoint) < MechanismSetpointConstants.armAllowableError;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_pivotAngle <= ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff) {
      /* Whenever the elevatarm is pivoting into danger area, clamp the elevator setpoint and always move elevator before pivoting arm. 
       * This prevents any end effector collision with the top of the drivebase
      */
      double elevatorClampedSetpoint = MathUtil.clamp(m_extensionMeters, 
              ElevatarmConstants.elevatorLowToGroundMinRetraction, ElevatarmConstants.elevatorForwardSoftLimit);

      m_commandToRun = 
            new InstantCommand(() -> 
            {
              s_Elevatarm.setElevatorPosition(elevatorClampedSetpoint);
              if (s_Elevatarm.getArmRotation2d().getRotations() > ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff) {
                s_Elevatarm.setArmPosition(ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff);
              } else {
                // safe to pivot if target is in danger zone & current in danger zone
                s_Elevatarm.setArmPosition(m_pivotAngle);
              }
            })
              .alongWith(new WaitUntilCommand(() -> {return this.isElevatorAtSetpoint(elevatorClampedSetpoint);}))
            .andThen(
              new InstantCommand(() -> s_Elevatarm.setArmPosition(m_pivotAngle))
              .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle)
              && this.isElevatorAtSetpoint(elevatorClampedSetpoint);}))
            )
      ;
    } else if (s_Elevatarm.getArmRotation2d().getRotations() <= ElevatarmConstants.elevatorMinRetractionInterferenceAngleCutoff) {
      /* When the elevatarm is not pivoting within danger area BUT it is currently in danger area, pivot arm before moving elevator. 
       * This prevents any end effector collision with the front of the drivebase
      */
      m_commandToRun = 
        new InstantCommand(() -> {
          s_Elevatarm.setArmPosition(m_pivotAngle);
          if (ElevatarmConstants.elevatorLowToGroundMinRetraction > m_extensionMeters){
            s_Elevatarm.setElevatorPosition(ElevatarmConstants.elevatorLowToGroundMinRetraction);
          }
          else {
            // safe to extend if currently in danger zone and elevator extension is globally safe
            s_Elevatarm.setElevatorPosition(m_extensionMeters);
          }
        })
              .alongWith(new WaitUntilCommand(() -> {return this.isElevatorAtSetpoint(m_extensionMeters)
                || this.isElevatorAtSetpoint(ElevatarmConstants.elevatorLowToGroundMinRetraction);}))
            .andThen(
              new InstantCommand(() -> s_Elevatarm.setElevatorPosition(m_extensionMeters))
              .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle)
              && this.isElevatorAtSetpoint(m_extensionMeters);}))
            )
      ;
    } else {
      /* Otherwise, pivot arm and extend elevator at the same time. */
      m_commandToRun = 
        new InstantCommand(() -> s_Elevatarm.setArmPosition(m_pivotAngle))
        .alongWith(new InstantCommand(() -> s_Elevatarm.setElevatorPosition(m_extensionMeters)))
        .alongWith(new WaitUntilCommand(() -> {return this.isArmAtSetpoint(m_pivotAngle) && this.isElevatorAtSetpoint(m_extensionMeters);}))
      ;
    }
    
    // If the climb is too high, stow the climb before moving the elevatarm.
    // The logic is only followed when pivoting acros the minimum angle for climb clearance
    if (
      (
        s_Climb.getPositionLeft() > ClimbConstants.climbMaxExtensionForElevatarmClearance
        || 
        s_Climb.getPositionRight() > ClimbConstants.climbMaxExtensionForElevatarmClearance)
    && 
    (
      (
      s_Elevatarm.getArmRotation2d().getRotations() < ElevatarmConstants.armCutoffAngleForClimbClearance
      && m_pivotAngle > ElevatarmConstants.armCutoffAngleForClimbClearance
      ) || 
      (
        s_Elevatarm.getArmRotation2d().getRotations() > ElevatarmConstants.armCutoffAngleForClimbClearance
        && m_pivotAngle < ElevatarmConstants.armCutoffAngleForClimbClearance
      )
    )
    ) {
      m_commandToRun = new ClimbPositionCommand(ClimbPosition.STOWED, s_Climb)
      .andThen(m_commandToRun);
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
    m_commandToRun.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }
}
