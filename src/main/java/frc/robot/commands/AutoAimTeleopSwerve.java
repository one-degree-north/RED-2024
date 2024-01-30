package frc.robot.commands;

import frc.lib.util.ShotCalculator;
import frc.lib.util.ShotCalculator.ShotData;
import frc.robot.Constants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoAimTeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private BooleanSupplier resetGyroSup;

    private SlewRateLimiter slewRateLimiterX;
    private SlewRateLimiter slewRateLimiterY;

    private ProfiledPIDController headingController;

    public AutoAimTeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier resetGyroSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.resetGyroSup = resetGyroSup;

        this.slewRateLimiterX = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterY = new SlewRateLimiter(TeleopConstants.rateLimitXY);

        this.headingController = new ProfiledPIDController(TeleopConstants.autoAimHeadingkP, 0, 0, 
            new TrapezoidProfile.Constraints(TeleopConstants.headingMaxVelRadPerSec, TeleopConstants.headingMaxAccelRadPerSec));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();

        double translationVal = MathUtil.applyDeadband(translation, TeleopConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe, TeleopConstants.stickDeadband);

        if (resetGyroSup.getAsBoolean()) {
            s_Swerve.zeroGyro();
        }

        // Convert joystick values to speeds
        translationVal = translationVal*Constants.Swerve.maxSpeed;
        strafeVal = strafeVal*Constants.Swerve.maxSpeed;

        ShotData targetShot = ShotCalculator.calculate(
          Constants.PathGenerationConstants.speakerPosition,
          s_Swerve.getPhotonPose().getTranslation(),
          new Translation2d(
            s_Swerve.getCurrentChassisSpeeds().vxMetersPerSecond,
            s_Swerve.getCurrentChassisSpeeds().vyMetersPerSecond
          ).rotateBy(s_Swerve.getPhotonPose().getRotation().unaryMinus()));
        


        /* Drive */
        translationVal = slewRateLimiterX.calculate(translationVal);
        strafeVal = slewRateLimiterY.calculate(strafeVal);
        double rotationVal = headingController.calculate(
          MathUtil.angleModulus(s_Swerve.getPhotonPose().getRotation().getRadians()), 
          MathUtil.angleModulus(MathUtil.angleModulus(targetShot.goalHeading().getRadians())));

        s_Swerve.drive(
            new Translation2d(
                translationVal, 
                strafeVal), 
            rotationVal, 
            true, 
            true
        );

        SmartDashboard.putNumber("Auto Aim Error", headingController.getPositionError());
    }
}