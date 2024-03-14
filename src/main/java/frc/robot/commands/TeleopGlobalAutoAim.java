package frc.robot.commands;

import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ShotCalculator.ShotData;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.MechanismSetpointConstants;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevatarm;
import frc.robot.subsystems.Shintake;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class TeleopGlobalAutoAim extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private BooleanSupplier resetGyroSup;

    private SlewRateLimiter slewRateLimiterX;
    private SlewRateLimiter slewRateLimiterY;

    private PIDController headingController;

    private Shintake s_Shintake;
    private Elevatarm s_Elevatarm;
    private Climb s_Climb;
    private Trigger m_justShootTrigger;

    public TeleopGlobalAutoAim(Swerve s_Swerve, Elevatarm s_Elevatarm, Shintake s_Shintake, Climb s_Climb, DoubleSupplier translationSup, DoubleSupplier strafeSup, Trigger resetGyroSup, Trigger justShootButton) {
        this.s_Swerve = s_Swerve;
        this.s_Elevatarm = s_Elevatarm;
        this.s_Shintake = s_Shintake;
        this.s_Climb = s_Climb;
        addRequirements(s_Swerve, s_Elevatarm, s_Shintake, s_Climb);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.resetGyroSup = resetGyroSup;
        this.m_justShootTrigger = justShootButton;

        this.slewRateLimiterX = new SlewRateLimiter(TeleopConstants.rateLimitXY);
        this.slewRateLimiterY = new SlewRateLimiter(TeleopConstants.rateLimitXY);

        this.headingController = new PIDController(TeleopConstants.autoAimHeadingkP, TeleopConstants.autoAimHeadingkI, TeleopConstants.autoAimHeadingkD);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(MechanismSetpointConstants.swerveRotationAllowableError);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();

        double translationVal = MathUtil.applyDeadband(translation, TeleopConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe, TeleopConstants.stickDeadband);

        if (resetGyroSup.getAsBoolean()) {
            s_Swerve.zeroHeading();
        }

        // Convert joystick values to speeds
        translationVal = translationVal*Constants.Swerve.maxSpeed;
        strafeVal = strafeVal*Constants.Swerve.maxSpeed;

        ShotData targetShot = s_Swerve.getShotData();
        
        /* Drive */
        translationVal = slewRateLimiterX.calculate(translationVal);
        strafeVal = slewRateLimiterY.calculate(strafeVal);
        double rotationVal = MathUtil.applyDeadband(headingController.calculate(
        // Use odometry-obtained rotation as measurement 
          MathUtil.angleModulus(s_Swerve.getPose().getRotation().getRadians()), 
          MathUtil.angleModulus(targetShot.goalHeading().getRadians())
        ), 
        TeleopConstants.stickDeadband*Constants.Swerve.maxAngularVelocity);

        s_Swerve.drive(
            new Translation2d(
                translationVal, 
                strafeVal), 
            rotationVal, 
            true, 
            false
        );

        // let shooter run the whole time this command is run
        s_Shintake.setShooterVelocityRPM(ShintakeConstants.shooterLeftRPM, ShintakeConstants.shooterRightRPM);
        // Move climb to save position if it is not already there
        if (s_Climb.getPositionLeft() > ClimbConstants.climbMaxExtensionForElevatarmClearance
        || s_Climb.getPositionRight() > ClimbConstants.climbMaxExtensionForElevatarmClearance) {
            s_Climb.setSetpointPositionLeft(MechanismSetpointConstants.climbStowedPosition);
            s_Climb.setSetpointPositionRight(MechanismSetpointConstants.climbStowedPosition);
        } else if (!m_justShootTrigger.getAsBoolean()) {
            s_Elevatarm.setElevatorPosition(MechanismSetpointConstants.elevatorGroundIntakePosition/2);
            s_Elevatarm.setArmPosition(s_Swerve.getShotData().clampedArmAngle());
        }

        else if (m_justShootTrigger.getAsBoolean()) {
            s_Elevatarm.setElevatorPosition(MechanismSetpointConstants.elevatorGroundIntakePosition/2);
            s_Elevatarm.setArmPosition(s_Swerve.getShotData().clampedArmAngle());
            s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed);
        }
        // if elevator is not at correct length, do this before running anything else with the elevator / arm
        
        // if justshoottrigger is pressed, just shoot
             // if the elevator is in the right position BUT arm is not at setpoint and swerve is not at setpoint
        // and swerve is not past the x position cutoff, set arm to auto aim angle
        // else if (
        // Math.abs(s_Elevatarm.getArmRotation2d().getRotations()-s_Swerve.getShotData().clampedArmAngle()) 
        // > MechanismSetpointConstants.armAllowableError
        // ||
        // Math.abs(
        //   MathUtil.angleModulus(
        //     s_Swerve.getShotData().goalHeading()
        //     .minus(s_Swerve.getPose().getRotation())
        //     .getRadians()
        //   )
        // )
        // > MechanismSetpointConstants.swerveRotationAllowableError
        // ||
        // s_Swerve.getShotData().effectiveRobotToSpeakerDist() 
        // > MechanismSetpointConstants.distanceCutoffToAutoScore
        // ||
        // Math.abs(s_Shintake.getLeftShooterVelocityRPM() - ShintakeConstants.shooterLeftRPM)
        // > MechanismSetpointConstants.flywheelVelocityAllowableError
        // ||
        // Math.abs(s_Shintake.getRightShooterVelocityRPM() - ShintakeConstants.shooterRightRPM)
        // > MechanismSetpointConstants.flywheelVelocityAllowableError
        // ||
        // Math.abs(s_Swerve.getTranslationalSpeed())
        // > MechanismSetpointConstants.allowableVelocityToAutoScore
        // ) {
        //     s_Elevatarm.setElevatorPosition(MechanismSetpointConstants.elevatorGroundIntakePosition);
        //     s_Elevatarm.setArmPosition(s_Swerve.getShotData().clampedArmAngle());

        // // if all subsystems are at setpoint, feed the note in to the shooter
        // } else {
        //     s_Elevatarm.setElevatorPosition(MechanismSetpointConstants.elevatorGroundIntakePosition);
        //     s_Elevatarm.setArmPosition(s_Swerve.getShotData().clampedArmAngle());
        //     s_Shintake.setIntakePercentSpeed(ShintakeConstants.intakePercentSpeed);
        // }


        SmartDashboard.putNumber("Auto Aim Rotation Error", headingController.getPositionError());


    }

    @Override
    public void end(boolean interrupted) {
        s_Shintake.stopAll();
    }


}