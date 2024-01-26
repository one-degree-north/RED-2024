package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private AutoScorePosition currentAutoScorePosition = AutoScorePosition.CENTER;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton goToPos = new JoystickButton(driver, XboxController.Button.kX.value);
    
    private final POVButton setCenterAutoScore = new POVButton(driver, 0);
    private final POVButton setLeftAutoScore = new POVButton(driver, 270);
    private final POVButton setRightAutoScore = new POVButton(driver, 90);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final LEDs s_LEDs = new LEDs(9, s_Swerve);
    

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    // TODO: implement LED indicators for auto starting pose
    private final Supplier<Pose2d> autoStartingPoseSupplier = 
        () -> {
            if (autoChooser.getSelected() != null)
                return PathPlannerAuto.
                    getStaringPoseFromAutoFile(autoChooser.getSelected().getName());
            return null;
        };

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Adding Autos */
        SmartDashboard.putData(autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> zeroGyro.getAsBoolean()
            )
        );

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

        // TODO: try removing method in lambda
        goToPos.whileTrue(new AutoScorePathfind(() ->{return getAutoScorePosition();}, s_Swerve));

        setCenterAutoScore.onTrue(
            new InstantCommand(() -> currentAutoScorePosition = AutoScorePosition.CENTER)
        );

        setLeftAutoScore.onTrue(
            new InstantCommand(() -> currentAutoScorePosition = AutoScorePosition.LEFT)
            );

        setRightAutoScore.onTrue(
            new InstantCommand(() -> currentAutoScorePosition = AutoScorePosition.RIGHT)
        );



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
