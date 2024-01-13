package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
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

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton goToPos = new JoystickButton(driver, XboxController.Button.kX.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final LEDSubsystem s_LEDSubsystem = new LEDSubsystem(9, 60);

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Adding Autos */
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    public void standardLEDBehavior() {
        s_LEDSubsystem.setDefaultCommand(new InstantCommand(() ->
        {
            if (s_Swerve.getSpeed() < Constants.Swerve.maxSpeed*0.9) {
                if (DriverStation.isTeleop())
                    s_LEDSubsystem.percentage(
                        () -> s_Swerve.getSpeed()/Constants.Swerve.maxSpeed, 
                        0, 23);
                else if (DriverStation.isAutonomous())
                    s_LEDSubsystem.percentageAuto(
                        () -> s_Swerve.getSpeed()/Constants.Swerve.maxSpeed, 
                        0, 23);
            }
            else if (s_Swerve.getSpeed() >= Constants.Swerve.maxSpeed*0.9) {
                if (DriverStation.isTeleop())
                    s_LEDSubsystem.flow(0, 23);
                else if (DriverStation.isAutonomous())
                    s_LEDSubsystem.flowAuto(0, 23);

            }
            
        }, s_LEDSubsystem));
    }

    public void disabledLEDBehavior() {

        // Aqua for calibrated, Blue for calibrated
        if (s_Swerve.getTagSeenSinceLastDisable())
            s_LEDSubsystem.setRangeStaticRGB(0, 0, 255, 0, 23);
        
        else s_LEDSubsystem.setRangeStaticRGB(0, 255, 255, 0, 23);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
    
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        goToPos.whileTrue(s_Swerve.goToPose(
            new Pose2d(12, 2, Rotation2d.fromDegrees(0))
        ));

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
