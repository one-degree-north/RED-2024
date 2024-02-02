package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.ShintakeCommand.ShintakeMode;
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
    private final JoystickButton autoAim = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve(); 
    private final Shintake s_Shintake = new Shintake();
 

    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("6 Note Auto");

    // TODO: implement LED indicators for auto starting pose
    public Supplier<Pose2d> autoStartingPoseSupplier = 
        () -> {
            if ((DriverStation.isFMSAttached() || DriverStation.isDSAttached()) 
            && (autoChooser.getSelected() != Commands.none()) 
            && (autoChooser.getSelected() != null))
                return s_Swerve.flipPose(PathPlannerAuto.
                    getStaringPoseFromAutoFile(autoChooser.getSelected().getName()));
            else return s_Swerve.getPose();
        };

    private final LEDs s_LEDs = new LEDs(9, s_Swerve, autoStartingPoseSupplier);
    

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
                () -> false,
                () -> zeroGyro.getAsBoolean()
            )
        );


        // Configure the button bindings
        configureButtonBindings();
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */


        autoAim.whileTrue(new AutoAimSpeakerTeleop(s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> zeroGyro.getAsBoolean()));

        intake.whileTrue(new ShintakeCommand(ShintakeMode.GROUND_INTAKE, s_Shintake));
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
