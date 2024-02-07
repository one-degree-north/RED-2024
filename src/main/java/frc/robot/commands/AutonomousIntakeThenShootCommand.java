// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShintakeCommand.ShintakeMode;
import frc.robot.subsystems.Elevatarm;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousIntakeThenShootCommand extends SequentialCommandGroup {
  private Shintake s_Shintake;
  private Swerve s_Swerve;
  private Elevatarm s_Elevatarm;
  private final double allowableError = 0.1;

  /** Creates a new AutonomousIntakeThenShootCommand. */
  public AutonomousIntakeThenShootCommand(Shintake shintake, Swerve swerve, Elevatarm elevatarm) {
    this.s_Shintake = shintake;
    this.s_Swerve = swerve;
    this.s_Elevatarm = elevatarm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // intake until note is intaked
      new ShintakeCommand(ShintakeMode.GROUND_INTAKE, s_Shintake)
      .onlyWhile(() -> !s_Shintake.isNoteIntaked()),

      // go to setpoints and only move on once reached
      new ParallelCommandGroup(
        new AutonomousSwerveAutoAimCommand(s_Swerve),
        new ElevatarmAutoAimCommand(s_Elevatarm, s_Swerve)
      ).until(() -> {
        return Math.abs(
          MathUtil.angleModulus(
            s_Swerve.getShotData().goalHeading()
            .minus(s_Swerve.getYaw())
            .getRadians()
          )
        ) 
        < allowableError
        &&
        Math.abs(
          s_Swerve.getShotData().clampedArmAngle()
          - s_Elevatarm.getArmRotation2d().getRotations()
        )
        < allowableError;
      }), 

      // continue tracking and shoot after reaching setpoints

      new ParallelRaceGroup(
        new ShintakeCommand(ShintakeMode.SHOOT, s_Shintake),
        new AutonomousSwerveAutoAimCommand(s_Swerve),
        new ElevatarmAutoAimCommand(s_Elevatarm, s_Swerve)
      )
    );


  }
}
