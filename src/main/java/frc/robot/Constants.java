package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    // INVERTS ARE IN SUBSYSTEMS THEMSELVES - NOT HERE

    public static final class TeleopConstants {
        public static final double stickDeadband = 0.1;
        public static final double rateLimitXY = 15;
        public static final double rateLimitTheta = 15 * Math.PI;

        // TODO: Tune PID
        public static final double headingkP = 0.2;
        public static final double headingkI = 0;
        public static final double headingkD = 0;
        public static final double headingMaxVelRadPerSec = AutoConstants.angularVelocityConstraint;
        public static final double headingMaxAccelRadPerSec = AutoConstants.angularAccelerationConstraint;

        public static final double autoAimHeadingkP = 3.0;
        public static final double autoAimHeadingkI = 0;
        public static final double autoAimHeadingkD = 0.1;
    }

    public static final class PathGenerationConstants {
        public static final Pose2d ampScoringPose = new Pose2d();

        public static final Translation2d speakerTranslation = new Translation2d(Units.inchesToMeters(0),
                Units.inchesToMeters(218.42));

        // This is always from the perspective of the driver (left and right flip
        // depending on alliance)
        public static final Pose2d leftSourceIntakingPose = new Pose2d();
        public static final Pose2d middleSourceIntakingPose = new Pose2d();
        public static final Pose2d rightSourceIntakingPose = new Pose2d();

        public static final Pose2d leftSpeakerScoringPose = new Pose2d();
        public static final Pose2d middleSpeakerScoringPose = new Pose2d();
        public static final Pose2d rightSpeakerScoringPose = new Pose2d();

    }

    public static final class MechanismSetpointConstants {
        // Elevator in meters, arm in rotations
        public static final double elevatorStowedPosition = 0;
        public static final double armStowedPosition = 0;

        public static final double elevatorGroundIntakePosition = 0;
        public static final double armGroundIntakePosition = 0;

        public static final double elevatorSourcePosition = 0;
        public static final double armSourcePosition = 0;

        public static final double elevatorAmpPosition = 0;
        public static final double armAmpPosition = 0;

        public static final double elevatorPreTrapPosition = 0;
        public static final double armPreTrapPosition = 0;

        public static final double elevatorTrapPosition = 0;
        public static final double armTrapPosition = 0;

        public static final double elevatorSpeakerPosition = elevatorGroundIntakePosition;

        // Climb in meters or meters per second
        public static final double climbHighPosition = 0;
        public static final double climbLowPosition = 0;
        public static final double climbStandardPosition = 0;
    }

    public static final class ElevatarmConstants {
        // Arm IDs
        public static final int armLeaderID = 0;
        public static final int armFollowerID = 0;
        public static final int armEncoderPort = 0;

        public static final double armAbsoluteEncoderAngleOffset = 0.0;
        public static final double armForwardSoftLimit = 0.0;
        public static final double armReverseSoftLimit = 0.0;

        // Arm MotionMagic gains
        public static final double armkP = 0.0;
        public static final double armkI = 0.0;
        public static final double armkD = 0.0;
        // Arm kG must be calculated with elevator locked fully extended
        public static final double armkG = 0.0;
        public static final double armkS = 0.0;
        public static final double armkV = 0.0;
        public static final double armkA = 0.0;
        // Rotations per second
        public static final double armCruiseVelocity = 0.0;
        public static final double armAcceleration = 0.0;

        public static final double armGearRatio = 120.0 / 1.0; // Max increase to 250:1

        public static final int elevatorID = 0;
        public static final int elevatorEncoderPort = 0;
        public static final int elevatorLimitMinPort = 0;
        public static final int elevatorLimitMaxPort = 0;

        public static final double elevatorAbsoluteEncoderDistanceOffset = 0.0;
        public static final double elevatorForwardSoftLimit = 0.0;
        public static final double elevatorReverseSoftLimit = 0.0;

        public static final double elevatorkP = 0.0;
        public static final double elevatorkI = 0.0;
        public static final double elevatorkD = 0.0;
        // Elevator kG must be calculated with arm locked straight upwards
        public static final double elevatorkG = 0.0;
        public static final double elevatorkS = 0.0;
        public static final double elevatorkV = 0.0;
        public static final double elevatorkA = 0.0;
        // Meters per second
        public static final double elevatorCruiseVelocity = 0.0;
        public static final double elevatorAcceleration = 0.0;

        public static final double elevatorSprocketDiameter = Units.inchesToMeters(1.432);

        public static final double elevatorIntegratedSensorToAbsoluteSensor = 5.0 / 1.0; // 5:1
        public static final double elevatorMechanismRotationsToMeters = elevatorSprocketDiameter * Math.PI;

        // Distance from pivot to center of fully retracted end effector
        public static final double minDistanceOfShintakeRelativeToPivot = Units.inchesToMeters(23.5);
        public static final double maxDistanceOfShintakeRelativeToPivot = elevatorForwardSoftLimit + minDistanceOfShintakeRelativeToPivot; 
            // Maximum should be 34.5

        // In meters
        public static final Translation2d positionOfPivotRelativeToOrigin = new Translation2d(-13.5, 7.6);

        // The minimum retraction of the elevator when it is less than the intereference
        // angle cutoff (in meters and relative to minimum possible retraction of elevator)
        public static final double elevatorLowToGroundMinRetraction = 0;
        // Lowest possible angle that the arm can be at without the end effector (fully
        // retracted) interfering with the drivetrain (in rotations)
        public static final double elevatorMinRetractionInterferenceAngleCutoff = 0;

    }

    public static final class ClimbConstants {
        public static final int leftClimbID = 0;
        public static final int rightClimbID = 0;

        public static final int leftClimbEncoderPort = 0;
        public static final int rightClimbEncoderPort = 0;

        public static final int leftPneumaticBreakPort1 = 0;
        public static final int leftPneumaticBreakPort2 = 0;

        public static final int rightPneumaticBreakPort1 = 0;
        public static final int rightPneumaticBreakPort2 = 0;

        // in meters
        public static final double leftClimbAbsoluteEncoderOffset = 0;
        public static final double rightClimbAbsoluteEncoderOffset = 0;

        public static final double climbAbsoluteSensorToMotorRatio = 0;
        public static final double climbIntegratedEncoderRotationsPerDistance = 0;

        public static final double climbPositionkP = 0.0;
        public static final double climbPositionkI = 0.0;
        public static final double climbPositionkD = 0.0;
        public static final double climbPositionkG = 0.0;
        public static final double climbPositionkS = 0.0;
        public static final double climbPositionkV = 0.0;
        public static final double climbPositionkA = 0.0;
        // Meters per second
        public static final double climbCruiseVelocity = 0.0;
        public static final double climbAcceleration = 0.0;

        public static final double climbVelocitykP = 0.0;
        public static final double climbVelocitykI = 0.0;
        public static final double climbVelocitykD = 0.0;
        public static final double climbVelocitykG = 0.0;
        public static final double climbVelocitykS = 0.0;
        public static final double climbVelocitykV = 0.0;
        public static final double climbVelocitykA = 0.0;

        public static final double climbForwardSoftLimit = 0.0;
        public static final double climbReverseSoftLimit = 0.0;

        // Meters per second
        public static final double climbStandardVelocity = 0.1;
    }

    public static final class ShintakeConstants {
        public static final int leftShooterID = 0;
        public static final int rightShooterID = 0;

        public static final int intakeID = 1;
        public static final int irSensorPort = 0;

        public static final double shooterkP = 0;
        public static final double shooterkI = 0;
        public static final double shooterkD = 0;
        public static final double shooterkS = 0;
        public static final double shooterkV = 0;
        public static final double shooterkA = 0;

        public static final double flywheelGearing = 1.0/1.0; // 1:1

        // Approximate time shooter takes to hit max velocity
        public static final double shooterRampTimeSeconds = 1;

        // Time taken for note to be shot after beam break is triggered
        public static final double shooterDelaySeconds = 0.5;

        // Time taken for note to be outtaked after beam break sensor is triggered for
        // amp and trap scoring
        public static final double ampAndTrapDelaySeconds = 1;

        // Time taken for intaking to stop after beam break sensor is triggered for
        // source intaking
        public static final double sourceIntakeDelaySeconds = 0.5;

        // For Intake
        public static final double outtakePercentSpeed = 0.1;
        // For shooter
        public static final double outtakeRPM = 300;

        // Regular shooter/intake preset velocities
        public static final double shooterLeftRPM = 6000;
        public static final double shooterRightRPM = 4000;
        public static final double intakePercentSpeed = 0.7;
    }

    // TODO: Tune slew rate limiter to driver's preferences (this is basically
    // acceleration)

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Comment out if not using supported module
        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);
        /* Drivetrain Constants */
        // This must be tuned to specific robot.
        public static final double trackWidth = Units.inchesToMeters(17.75);
        public static final double wheelBase = Units.inchesToMeters(17.75);
        public static final double drivebaseRadius = Math.hypot(trackWidth, wheelBase) / 2.0;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        // DISREGARD IF USING SUPPORTED MODULES
        // This must be tuned to specific robot.
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        // DISREGARD IF USING SUPPORTED MODULES
        // This must be tuned to specific robot.
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        // DISREGARD IF USING SUPPORTED MODULES
        // This must be tuned to specific robot.
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        // DISREGARD IF USING SUPPORTED MODULES
        // This must be tuned to specific robot. We can do this through Phoenix Tuner
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        // TODO: This must be tuned to specific robot. We can do this through Phoenix
        // Tuner
        public static final double driveKP = 0.135;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        // kF depricated in Phoenix 6
        // public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // This must be tuned to specific robot. We can do this by locking the rotation
        // gears and using SYSID.
        public static final double driveKS = (0.12698);
        public static final double driveKV = (2.1248);
        public static final double driveKA = (0.14197); // TODO: Used to divide by 12 to convert

        /* Swerve Profiling Values */
        // This must be tuned to specific robot.
        /** Meters per Second */
        public static final double maxSpeed = 5.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 6 * Math.PI;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        // These must be tuned to specific robot.

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(33.838);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.269);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.277);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.927);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static class VisionConstants {
        // These must be tuned to specific robot.

        // 3D Translation from AprilTag camera to center of robot
        // TODO: Get offset values
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(9), Units.inchesToMeters(8), Units.inchesToMeters(8.75)),
                new Rotation3d(0.0, -Units.degreesToRadians(30), -Units.degreesToRadians(45)));
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_2 = new Transform3d(
                new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(-8), Units.inchesToMeters(8.75)),
                new Rotation3d(0.0, -Units.degreesToRadians(30), Units.degreesToRadians(135)));

        // Calculated field length for 2023 game (used to circumvent "flipping tags" as
        // well as mirror coordinates for red/blue alliance)
        public static final double FIELD_LENGTH_METERS = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
                .getFieldLength();
        public static final double FIELD_WIDTH_METERS = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
                .getFieldWidth();

        // Minimum target ambiguity. Targets with higher ambiguity will be discarded
        // TODO: Test ambiguity
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    }

    public static final class AutoConstants {
        // TODO: These must be tuned to specific robot
        public static final double velocityConstraint = Swerve.maxSpeed;
        public static final double accelerationConstraint = Swerve.maxSpeed;
        public static final double angularVelocityConstraint = Swerve.maxAngularVelocity;
        public static final double angularAccelerationConstraint = Swerve.maxAngularVelocity * 2;
    }
}
