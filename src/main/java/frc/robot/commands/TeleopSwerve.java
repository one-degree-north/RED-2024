package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter slewRateLimiterX;
    private SlewRateLimiter slewRateLimiterY;
    private SlewRateLimiter slewRateLimiterR;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        this.slewRateLimiterX = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterY = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterR = new SlewRateLimiter(TeleopConstants.rateLimitTheta);
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



        /* Drive */
        s_Swerve.drive(
            new Translation2d(
                slewRateLimiterX.calculate(translationVal*Constants.Swerve.maxSpeed), 
                slewRateLimiterY.calculate(strafeVal*Constants.Swerve.maxSpeed)), 
            slewRateLimiterR.calculate(rotationVal * Constants.Swerve.maxAngularVelocity), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}