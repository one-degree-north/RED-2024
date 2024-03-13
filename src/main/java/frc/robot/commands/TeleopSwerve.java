package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier resetGyroSup;

    private SlewRateLimiter slewRateLimiterX;
    private SlewRateLimiter slewRateLimiterY;
    private SlewRateLimiter slewRateLimiterR;

    private ProfiledPIDController headingController;
    private boolean correctionEnabled = false;
    private double lastHeadingRadians = 0;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier resetGyroSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.resetGyroSup = resetGyroSup;

        this.slewRateLimiterX = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterY = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterR = new SlewRateLimiter(TeleopConstants.rateLimitTheta);

        this.headingController = new ProfiledPIDController(TeleopConstants.headingkP, TeleopConstants.headingkI, TeleopConstants.headingkD, 
            new TrapezoidProfile.Constraints(TeleopConstants.headingMaxVelRadPerSec, TeleopConstants.headingMaxAccelRadPerSec));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();
        double rotation = rotationSup.getAsDouble();

        double translationVal = MathUtil.applyDeadband(translation, TeleopConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe, TeleopConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation, TeleopConstants.stickDeadband);

        if (resetGyroSup.getAsBoolean()) {
            lastHeadingRadians = 0;
            s_Swerve.zeroHeading();
        }

        // Convert joystick values to speeds
        translationVal = translationVal*Constants.Swerve.maxSpeed;
        strafeVal = strafeVal*Constants.Swerve.maxSpeed;
        rotationVal = rotationVal*Constants.Swerve.maxAngularVelocity;

        if (Math.abs(rotationVal) < 0.01 && (Math.abs(translationVal) > 0.01 || Math.abs(strafeVal) > 0.01)) {
            if (!correctionEnabled) {
                lastHeadingRadians = MathUtil.angleModulus(s_Swerve.getYaw().getRadians());
                correctionEnabled = true;
            }
            rotationVal = headingController.calculate(MathUtil.angleModulus(
                s_Swerve.getYaw().getRadians()), 
                lastHeadingRadians);
        } else {
            correctionEnabled = false;
        }

        /* Drive */
        translationVal = slewRateLimiterX.calculate(translationVal);
        strafeVal = slewRateLimiterY.calculate(strafeVal);

        // Still calculate rotation limited value even if it is not used
        double rotationLimitedVal = slewRateLimiterR.calculate(rotationVal);
        rotationVal = correctionEnabled ? rotationVal : rotationLimitedVal;

        s_Swerve.drive(
            new Translation2d(
                translationVal, 
                strafeVal), 
            rotationVal, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}