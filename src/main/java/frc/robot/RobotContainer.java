package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.Constants.PathGenerationConstants;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.commands.*;
import frc.robot.commands.AutoClimbPathfind.AutoClimbPosition;
import frc.robot.commands.AutoIntakePathfind.AutoIntakePosition;
import frc.robot.commands.autocommands.AutonomousShootContinuousCommand;
import frc.robot.commands.climbcommands.ClimbPositionCommand;
import frc.robot.commands.climbcommands.ClimbVelocityCommand;
import frc.robot.commands.climbcommands.ClimbPositionCommand.ClimbPosition;
import frc.robot.commands.climbcommands.ClimbVelocityCommand.ClimbToMove;
import frc.robot.commands.elevatarmcommands.ArmManualControlCommand;
import frc.robot.commands.elevatarmcommands.ElevatarmCommand;
import frc.robot.commands.elevatarmcommands.ElevatorManualControlCommand;
import frc.robot.commands.shintakecommands.ShintakeCommand;
import frc.robot.commands.shintakecommands.ShintakeCommand.ShintakeMode;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandPS5Controller mainController = new CommandPS5Controller(0);
    private final CommandGenericHID buttonBoard = new CommandGenericHID(1);


    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    private final Shintake s_Shintake = new Shintake();
    private final Elevatarm s_Elevatarm = new Elevatarm();
    // private final Climb s_Climb = new Climb();

    /* Auto Chooser */
    // change default auto
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("6 Note Auto");

    // KNOWN LIMITATION: Will return error if "None" command is selected
    public Supplier<Pose2d> autoStartingPoseSupplier = 
        () -> {
            if ((DriverStation.isFMSAttached() || DriverStation.isDSAttached()) 
            && (autoChooser.getSelected() != Commands.none()) 
            && (autoChooser.getSelected() != null))
                return AllianceFlipUtil.flipPose(PathPlannerAuto.
                    getStaringPoseFromAutoFile(autoChooser.getSelected().getName()));
            else return s_Swerve.getPose();
        };

    private final LEDs s_LEDs = new LEDs(9, s_Swerve, autoStartingPoseSupplier, s_Shintake, s_Elevatarm, null);

    private AutoIntakePosition selectedIntakePosition = AutoIntakePosition.CENTER;
    private AutoClimbPosition selectedClimbPosition = AutoClimbPosition.CENTER;
    
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Adding Autos */

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -mainController.getLeftY(), 
                () -> -mainController.getLeftX(), 
                () -> -mainController.getRightX(), 
                () -> false,
                mainController.touchpad()
            )
        );

        // s_Elevatarm.setDefaultCommand(
        //     new RepeatCommand(
        //         Commands.either(
        //             new ElevatarmCommand(
        //                 MechanismSetpointConstants.armGroundIntakePosition, 
        //                 MechanismSetpointConstants.elevatorGroundIntakePosition, 
        //                 s_Elevatarm
        //             ),

        //             new ElevatarmCommand(
        //                 MechanismSetpointConstants.armStowedPosition, 
        //                 MechanismSetpointConstants.elevatorStowedPosition, 
        //                 s_Elevatarm
        //             ),

        //             () -> {
        //                 return s_Swerve.isInClimbZone();
        //             }
        //         )
        //     )
        // );

        s_Shintake.setDefaultCommand(
            new InstantCommand(() -> s_Shintake.stopAll(), s_Shintake)
            .until(() -> false)
        );

        // Configure the button bindings
        configureButtonBindings();
        configureNamedCommands();
        configureSmartDashboardCommands();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        // TESTING MODE BUTTON BINDS
        mainController.povUp().whileTrue(
            new ArmManualControlCommand(0.2, s_Elevatarm)
        );
        mainController.povDown().whileTrue(
            new ArmManualControlCommand(-0.2, s_Elevatarm)
        );

        mainController.povRight().whileTrue(
            new ElevatorManualControlCommand(0.2, s_Elevatarm)
        );

        mainController.povLeft().whileTrue(
            new ElevatorManualControlCommand(-0.2, s_Elevatarm)
        );

        mainController.R2().onTrue(
            new InstantCommand(() -> s_Shintake.setShooterVelocityRPM(
                ShintakeConstants.shooterLeftRPM, ShintakeConstants.shooterRightRPM))
        );

        mainController.R2().onFalse(
            new InstantCommand(() -> s_Shintake.stopShooter())
        );

        mainController.R1().onTrue(
            new InstantCommand(() -> s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed))
        );

        mainController.triangle().onTrue(
            new InstantCommand(
                () -> s_Elevatarm.setElevatorPosition( 
                    MechanismSetpointConstants.elevatorGroundIntakePosition)
            )
        );

        // // REAL MATCH BUTTON BINDS
        // // Ground intake
        // mainController.L2().whileTrue(
        //     Commands.parallel(
        //         new ElevatarmCommand(
        //             MechanismSetpointConstants.armGroundIntakePosition, 
        //             MechanismSetpointConstants.elevatorGroundIntakePosition, 
        //             s_Elevatarm),
        //         new ShintakeCommand(ShintakeMode.GROUND_INTAKE, s_Shintake, true)
        //     )
        // );

        // // Global speaker shoot
        // mainController.R2().whileTrue(
        //     new TeleopGlobalAutoAim(s_Swerve, s_Elevatarm, s_Shintake, 
        //         () -> -mainController.getLeftY(), 
        //         () -> -mainController.getLeftX(), 
        //         mainController.touchpad()
        //     )
        // );

        // // Source intake
        // mainController.L1().whileTrue(
        //     Commands.sequence(
        //         Commands.race(
        //             new AutoIntakePathfind(() -> {return selectedIntakePosition;}, s_Swerve),
        //             s_Elevatarm.getDefaultCommand(),
        //             s_Shintake.getDefaultCommand()
        //         ),
        //         new ElevatarmCommand(
        //             MechanismSetpointConstants.armSourcePosition, 
        //             MechanismSetpointConstants.elevatorSourcePosition, 
        //             s_Elevatarm),
        //         new ShintakeCommand(ShintakeMode.SOURCE_INTAKE, s_Shintake, true)
        //     )
        // );

        // // Amp score
        // mainController.R1().whileTrue(
        //     Commands.sequence(
        //         Commands.race(
        //             s_Swerve.goToPose(PathGenerationConstants.ampScoringPose, 0, 0),
        //             s_Elevatarm.getDefaultCommand(),
        //             s_Shintake.getDefaultCommand()
        //         ),
        //         new ElevatarmCommand(
        //             MechanismSetpointConstants.armAmpPosition, 
        //             MechanismSetpointConstants.elevatorAmpPosition, 
        //             s_Elevatarm),
        //         new ShintakeCommand(ShintakeMode.AMP_AND_TRAP, s_Shintake, true)
        //     )
        // );

        // mainController.cross().whileTrue(
        //     Commands.sequence(
        //         new InstantCommand(
        //             () -> s_Climb.disablePneumaticBreak()
        //         ),
        //         Commands.race(
        //             new AutoClimbPathfind(() -> {return selectedClimbPosition;}, s_Swerve),
        //             new ElevatarmCommand(
        //             MechanismSetpointConstants.armGroundIntakePosition, 
        //             MechanismSetpointConstants.elevatorGroundIntakePosition, 
        //             s_Elevatarm),
        //             s_Shintake.getDefaultCommand()
        //         ),
        //         Commands.parallel(
        //             new ClimbPositionCommand(ClimbPosition.MIDDLE, s_Climb),
        //             new ElevatarmCommand(
        //                 MechanismSetpointConstants.armPreTrapPosition, 
        //                 MechanismSetpointConstants.elevatorPreTrapPosition, 
        //                 s_Elevatarm)
        //         ),
        //         new TeleopSwerve(s_Swerve, () -> 0.1, () -> 0, () -> 0, () -> true, () -> false)
        //             .raceWith(Commands.waitSeconds(0.2)),
        //         new ClimbVelocityCommand(-ClimbConstants.climbStandardVelocity, ClimbToMove.BOTH, s_Climb)
        //             .until(() -> 
        //             s_Climb.getPositionLeft() <= MechanismSetpointConstants.climbStowedPosition
        //             || s_Climb.getPositionRight() <= MechanismSetpointConstants.climbStowedPosition),
        //         new ElevatarmCommand(
        //             MechanismSetpointConstants.armTrapPosition, 
        //             MechanismSetpointConstants.elevatorTrapPosition, 
        //             s_Elevatarm),
        //         new ShintakeCommand(
        //             ShintakeMode.AMP_AND_TRAP, 
        //             s_Shintake, 
        //             true),
        //         new InstantCommand(
        //             () -> {
        //                 s_Climb.enablePneumaticBreak();
        //                 s_Climb.disableClimbMotors(); 
        //             }
        //         )
        //     )
        // );



        // // Button board bindings
        // buttonBoard.button(0).onTrue(new InstantCommand(() -> selectedIntakePosition = AutoIntakePosition.LEFT));
        // buttonBoard.button(1).onTrue(new InstantCommand(() -> selectedIntakePosition = AutoIntakePosition.CENTER));
        // buttonBoard.button(2).onTrue(new InstantCommand(() -> selectedIntakePosition = AutoIntakePosition.RIGHT));

        // buttonBoard.button(3).onTrue(new InstantCommand(() -> selectedClimbPosition = AutoClimbPosition.LEFT));
        // buttonBoard.button(4).onTrue(new InstantCommand(() -> selectedClimbPosition = AutoClimbPosition.CENTER));
        // buttonBoard.button(5).onTrue(new InstantCommand(() -> selectedClimbPosition = AutoClimbPosition.RIGHT));

        // buttonBoard.button(6).onTrue(
        //     Commands.parallel(
        //         new InstantCommand(() -> s_Climb.disablePneumaticBreak()),
        //         new ClimbPositionCommand(ClimbPosition.LEFTHIGH, s_Climb)
        //     )
        // );
        // buttonBoard.button(6).onFalse(
        //     Commands.sequence(
        //         new ClimbVelocityCommand(-ClimbConstants.climbStandardVelocity, ClimbToMove.BOTH, s_Climb)
        //             .until(() -> 
        //             s_Climb.getPositionLeft() <= MechanismSetpointConstants.climbStowedPosition
        //             || s_Climb.getPositionRight() <= MechanismSetpointConstants.climbStowedPosition),
        //         new InstantCommand(() -> s_Climb.enablePneumaticBreak())
        //     )
        // );

        // buttonBoard.button(7).onTrue(
        //     Commands.parallel(
        //         new InstantCommand(() -> s_Climb.disablePneumaticBreak()),
        //         new ClimbPositionCommand(ClimbPosition.MIDDLE, s_Climb)
        //     )
        // );
        // buttonBoard.button(7).onFalse(
        //     Commands.sequence(
        //         new ClimbVelocityCommand(-ClimbConstants.climbStandardVelocity, ClimbToMove.BOTH, s_Climb)
        //             .until(() -> 
        //             s_Climb.getPositionLeft() <= MechanismSetpointConstants.climbStowedPosition
        //             || s_Climb.getPositionRight() <= MechanismSetpointConstants.climbStowedPosition),
        //         new InstantCommand(() -> s_Climb.enablePneumaticBreak())
        //     )
        // );

        // buttonBoard.button(8).onTrue(
        //     Commands.parallel(
        //         new InstantCommand(() -> s_Climb.disablePneumaticBreak()),
        //         new ClimbPositionCommand(ClimbPosition.RIGHTHIGH, s_Climb)
        //     )
        // );
        // buttonBoard.button(8).onFalse(
        //     Commands.sequence(
        //         new ClimbVelocityCommand(-ClimbConstants.climbStandardVelocity, ClimbToMove.BOTH, s_Climb)
        //             .until(() -> 
        //             s_Climb.getPositionLeft() <= MechanismSetpointConstants.climbStowedPosition
        //             || s_Climb.getPositionRight() <= MechanismSetpointConstants.climbStowedPosition),
        //         new InstantCommand(() -> s_Climb.enablePneumaticBreak())
        //     )
        // );

        // buttonBoard.button(9).onTrue(
        //     // set to source high mode
        //     new InstantCommand(() -> s_LEDs.setIntakingLEDs(true))
        // );

        // buttonBoard.button(10).onTrue(
        //     // set to ground mode
        //     new InstantCommand(() -> s_LEDs.setIntakingLEDs(false))
        // );
        
    }

    private void configureSmartDashboardCommands() {
        SmartDashboard.putData(autoChooser);

        // SmartDashboard.putData(
        //     "Enable Compressor",
        //     new InstantCommand(() -> s_Climb.enableCompressor())
        // );

        // SmartDashboard.putData(
        //     "Disable Compressor",
        //     new InstantCommand(() -> s_Climb.disableCompressor())
        // );

        SmartDashboard.putData(
            "Home Position", 
            new ElevatarmCommand(
                    MechanismSetpointConstants.armStowedPosition, 
                    MechanismSetpointConstants.elevatorStowedPosition, 
                    s_Elevatarm
            )
        );

        SmartDashboard.putData(
            "Ground Intake Position", 
            new ElevatarmCommand(
                    MechanismSetpointConstants.armGroundIntakePosition, 
                    MechanismSetpointConstants.elevatorGroundIntakePosition, 
                    s_Elevatarm
            )
        );

        SmartDashboard.putData(
            "Source Intake Position", 
            new ElevatarmCommand(
                    MechanismSetpointConstants.armSourcePosition, 
                    MechanismSetpointConstants.elevatorSourcePosition, 
                    s_Elevatarm
            )
        );

        SmartDashboard.putData(
            "Amp Scoring Position", 
            new ElevatarmCommand(
                    MechanismSetpointConstants.armAmpPosition, 
                    MechanismSetpointConstants.elevatorAmpPosition, 
                    s_Elevatarm
            )
        );

        SmartDashboard.putData(
            "Trap Position", 
            new ElevatarmCommand(
                    MechanismSetpointConstants.armTrapPosition, 
                    MechanismSetpointConstants.elevatorTrapPosition, 
                    s_Elevatarm
            )
        );

        SmartDashboard.putData(
            "Pre Trap Position",
            new ElevatarmCommand(
                    MechanismSetpointConstants.armPreTrapPosition, 
                    MechanismSetpointConstants.elevatorPreTrapPosition, 
                    s_Elevatarm
            )
        );

        // SmartDashboard.putData(
        //     "Climb Middle Position", 
        //     new ClimbPositionCommand(ClimbPosition.MIDDLE, s_Climb)
        // );

        // SmartDashboard.putData(
        //     "Climb Left High Position", 
        //     new ClimbPositionCommand(ClimbPosition.LEFTHIGH, s_Climb)
        // );

        // SmartDashboard.putData(
        //     "Climb Right High Position", 
        //     new ClimbPositionCommand(ClimbPosition.RIGHTHIGH, s_Climb)
        // );

        // SmartDashboard.putData(
        //     "Climb Stowed Position", 
        //     new ClimbPositionCommand(ClimbPosition.STOWED, s_Climb)
        // );

        // SmartDashboard.putData(
        //     "Toggle Pneumatic Break",
        //     new InstantCommand(() -> s_Climb.togglePneumaticBreak())
        // );

        SmartDashboard.putData(
            "Run Ground Intake",
            new ShintakeCommand(ShintakeMode.GROUND_INTAKE
                , s_Shintake, true)
        );

        // Tune source intake delay
        SmartDashboard.putData(
            "Run Source Intake",
            new ShintakeCommand(ShintakeMode.SOURCE_INTAKE
                , s_Shintake, true)
        );

        // Tune shooter ramp time, shooter delay
        SmartDashboard.putData(
            "Run Shooter",
            new ShintakeCommand(ShintakeMode.SHOOT
                , s_Shintake, true)
        );
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("AutonomousShootContinuousCommand", new AutonomousShootContinuousCommand(s_Shintake, s_Swerve, s_Elevatarm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }

}
