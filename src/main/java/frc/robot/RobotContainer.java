package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.AutoScorePathfind.AutoScorePosition;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private AutoScorePosition currentAutoScorePosition = AutoScorePosition.CENTER;

    /* Driver Buttons */
    private final JoystickButton reverse = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve(); 
 

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("6 Note");

    // TODO: implement LED indicators for auto starting pose
    public Supplier<Pose2d> autoStartingPoseSupplier = 
        () -> {
            if (autoChooser.getSelected() != Commands.none() 
            && autoChooser.getSelected() != null)
                return PathPlannerAuto.
                    getStaringPoseFromAutoFile(autoChooser.getSelected().getName());
            else return s_Swerve.getPose();
        };

    private final LEDs s_LEDs = new LEDs(9, s_Swerve, autoStartingPoseSupplier);

    private final Shintake s_Shintake = new Shintake();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Adding Autos */
        SmartDashboard.putData(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    public AutoScorePosition getAutoScorePosition() {
        return currentAutoScorePosition;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        shoot.whileTrue(new ShootCommand(6000, 4000, s_Shintake));
        reverse.whileTrue(new ShootCommand(-300, -300, s_Shintake));
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
