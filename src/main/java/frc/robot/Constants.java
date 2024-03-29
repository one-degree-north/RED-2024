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
    public static final boolean tuningMode = false;

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

        public static final double autoAimHeadingkP = 3;
        public static final double autoAimHeadingkI = 0;
        public static final double autoAimHeadingkD = 0;
    }

    public static final class PathGenerationConstants {
        public static final Pose2d ampScoringPose = new Pose2d(1.523, 7.821, Rotation2d.fromDegrees(-90.35));

        public static final Translation2d speakerTranslation = new Translation2d(Units.inchesToMeters(4),
                Units.inchesToMeters(218.42));

        // This is always from the perspective of the driver (left and right flip
        // depending on alliance)
        public static final Pose2d leftSourceIntakingPose = new Pose2d();
        public static final Pose2d middleSourceIntakingPose = new Pose2d();
        public static final Pose2d rightSourceIntakingPose = new Pose2d();

        // These are only used for trap scoring
        public static final Pose2d leftClimbPreHookPose = new Pose2d();
        public static final Pose2d middleClimbPreHookPose = new Pose2d();
        public static final Pose2d rightClimbPreHookPose = new Pose2d();

    }

    public static final class MechanismSetpointConstants {
        public static final double armAllowableError = 0.005;
        public static final double elevatorAllowableError = 0.06;
        public static final double climbAllowableError = 0.05;
        public static final double swerveRotationAllowableError = 0.05;
        public static final double flywheelVelocityAllowableError = 150; // RPM

        // Elevator in meters, arm in rotations
        public static final double elevatorStowedPosition = 0;
        public static final double armStowedPosition = 0.06;

        public static final double elevatorGroundIntakePosition = ElevatarmConstants.elevatorLowToGroundMinRetraction;
        public static final double armGroundIntakePosition = 0;

        public static final double elevatorSourcePosition = 0;
        public static final double armSourcePosition = 0;

        public static final double elevatorAmpPosition = 0.11055;
        public static final double armAmpPosition = 0.2421875;

        public static final double elevatorPreTrapPosition = 0;
        public static final double armPreTrapPosition = 0;

        public static final double elevatorTrapPosition = 0;
        public static final double armTrapPosition = 0;

        public static final double elevatorSpeakerPosition = elevatorGroundIntakePosition;

        // Climb in meters or meters per second

        // high pos is the pos needed to grab chain with low hook
        public static final double climbHighPosition = 0;

        // low pos is the bare minimum pos needed to grab chain with high hook
        public static final double climbLowPosition = 0;

        // standard pos is the height needed to grab chain with high hook on the high side of the chain
        public static final double climbStandardPosition = 0;

        // stowed pos is where the climb should be during the majority of the match
        public static final double climbStowedPosition = 0;

        // What x position should the robot be less than such that it can auto aim+score on speaker?
        // This gets translated by alliance
        public static final double distanceCutoffToAutoScore = 6;

        public static final double yDistanceDifferenceToAutoScore = 2.2;
        
        // in meters per second
        public static final double allowableVelocityToAutoScore = 1;
    }

    public static final class ElevatarmConstants {
        // Arm IDs
        public static final int armLeaderID = 13;
        public static final int armFollowerID = 14;
        public static final int armEncoderPort = 0;
        public static final int elevatarmLockSwitchPort = 9;

        public static final double armAbsoluteEncoderAngleOffset = -0.313;
        public static final double armForwardSoftLimit = 0.35;
        public static final double armReverseSoftLimit = 0;

        // Arm MotionMagic gains
        public static final double armkP = 69.457;
        public static final double armkI = 0.0;
        public static final double armkD = 24.131;
        // Arm kG must be calculated with elevator locked fully extended
        public static final double armkG = 0.49811;
        public static final double armkS = 0.19043;
        public static final double armkV = 13.203;
        public static final double armkA = 1.565;
        // Rotations per second
        public static final double armCruiseVelocity = 0.25;
        public static final double armAcceleration = 0.5;

        public static final double armGearRatio = 120.0 / 1.0; // Max increase to 250:1

        public static final int elevatorID = 15;
        public static final int elevatorEncoderPort = 1;

        public static final double elevatorAbsoluteEncoderDistanceOffset = -0.001561;
        public static final double elevatorForwardSoftLimit = 0.21277;
        public static final double elevatorReverseSoftLimit = 0;

        public static final double elevatorkP = 0.0191;
        public static final double elevatorkI = 0.0;
        public static final double elevatorkD = 0.0;
        // Elevator kG must be calculated with arm locked straight upwards
        public static final double elevatorkG = 0.35;
        public static final double elevatorkS = 0.15;
        public static final double elevatorkV = 0.625;
        public static final double elevatorkA = 0.0;
        // Meters per second
        public static final double elevatorCruiseVelocity = 0.3;
        public static final double elevatorAcceleration = 0.5;

        public static final double elevatorOutputDiameter = Units.inchesToMeters(1.432);

        public static final double elevatorIntegratedSensorToAbsoluteSensorRatio = 5.0 / 1.0; // 5:1
        public static final double elevatorMechanismRotationsToMetersRatio = 1 / (elevatorOutputDiameter * Math.PI);

        // Distance from pivot to center of fully retracted end effector
        public static final double minDistanceOfShintakeRelativeToPivot = Units.inchesToMeters(25);
        public static final double maxDistanceOfShintakeRelativeToPivot = elevatorForwardSoftLimit + minDistanceOfShintakeRelativeToPivot; 
            // Maximum should be 34 inches

        // In meters
        public static final Translation2d positionOfPivotRelativeToOrigin = new Translation2d(-13.5, 7.6);

        // The minimum retraction of the elevator when it is less than the intereference
        // angle cutoff (in meters and relative to minimum possible retraction of elevator)
        public static final double elevatorLowToGroundMinRetraction = 0.109;
        // Lowest possible angle that the arm can be at without the end effector (fully
        // retracted) interfering with the drivetrain (in rotations)
        public static final double elevatorMinRetractionInterferenceAngleCutoff = 0.055;


        public static final double armCutoffAngleForClimbClearance = 0.167;

    }

    public static final class ClimbConstants {
        public static final int leftClimbID = 18;
        public static final int rightClimbID = 19;

        public static final int leftClimbEncoderPort = 4;
        public static final int rightClimbEncoderPort = 5;

        public static final int leftPneumaticBreakPort1 = 0;
        public static final int leftPneumaticBreakPort2 = 1;

        public static final int rightPneumaticBreakPort1 = 8;
        public static final int rightPneumaticBreakPort2 = 9;

        // in meters
        public static final double leftClimbAbsoluteEncoderOffset = 0.068;
        public static final double rightClimbAbsoluteEncoderOffset = -0.08;

        public static final double climbOutputDiameter = Units.inchesToMeters(1.25);
        public static final double climbIntegratedSensorToAbsoluteSensorRatio = 10.5/1.0;
        public static final double climbMechanismRotationsToMetersRatio = 1 / (climbOutputDiameter * Math.PI);

        public static final double climbPositionkP = 48.088;
        public static final double climbPositionkI = 0.0;
        public static final double climbPositionkD = 1.4743;
        public static final double climbPositionkG = 0.0;
        public static final double climbPositionkS = 0.25343;
        public static final double climbPositionkV = 1.1985;
        public static final double climbPositionkA = 0.019932;
        // Meters per second
        public static final double climbCruiseVelocity = 0.1;
        public static final double climbAcceleration = 0.1;

        public static final double climbVelocitykP = 0.86493;
        public static final double climbVelocitykI = 0.0;
        public static final double climbVelocitykD = 0.0;
        public static final double climbVelocitykG = 0.0;
        public static final double climbVelocitykS = 0.25343;
        public static final double climbVelocitykV = 1.1985;
        public static final double climbVelocitykA = 0.019932;

        public static final double climbForwardSoftLimit = 100;
        public static final double climbReverseSoftLimit = -100;

        public static final double climbMaxExtensionForElevatarmClearance = 0.0007;

        // Meters per second
        public static final double climbStandardVelocity = 0.05;
    }

    public static final class ShintakeConstants {
        public static final int leftShooterID = 17;
        public static final int rightShooterID = 16;

        public static final int intakeID = 1;
        public static final int irSensorPort = 3;

        public static final double leftShooterkP = 0.10011;
        public static final double leftShooterkI = 0;
        public static final double leftShooterkD = 0;
        public static final double leftShooterkS = 0.21867;
        public static final double leftShooterkV = 0.11989;
        public static final double leftShooterkA = 0.0101;

        public static final double rightShooterkP = 0.11627;
        public static final double rightShooterkI = 0;
        public static final double rightShooterkD = 0;
        public static final double rightShooterkS = 0.19384;
        public static final double rightShooterkV = 0.11783;
        public static final double rightShooterkA = 0.014849;

        public static final double flywheelGearing = 1.0/1.0; // 1:1

        // Approximate time shooter takes to hit max velocity
        public static final double shooterTimeoutRampTimeSeconds = 1;

        // Time taken for note to be shot after beam break is triggered
        public static final double shooterDelaySeconds = 0.5;

        // Time taken for note to be outtaked after beam break sensor is triggered for
        // amp and trap scoring
        public static final double ampAndTrapDelaySeconds = 0.7;

        // Time taken for intaking to stop after beam break sensor is triggered for
        // source intaking
        public static final double sourceIntakeDelaySeconds = 0.5;

        // For Intake
        public static final double outtakePercentSpeed = 0.3;
        // For shooter
        public static final double outtakeRPM = 500;

        // Regular shooter/intake preset velocities
        public static final double shooterLeftRPM = 5000;
        public static final double shooterRightRPM = 4000;

        public static final double throwNoteRPM = 2000;
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
        public static final double trackWidth = Units.inchesToMeters(22.5);
        public static final double wheelBase = Units.inchesToMeters(22.5);
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
        public static final double driveKS = (0.40011);
        public static final double driveKV = (0.74073);
        public static final double driveKA = (0.015159);

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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(35.68);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.445);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(172.089);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(159.345);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static class VisionConstants {
        // These must be tuned to specific robot.

        // 3D Translation from AprilTag camera to center of robot
        // TODO: Get offset values

        // Start from front left and go clockwise

        //FL
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.96), Units.inchesToMeters(11.47), Units.inchesToMeters(8.32)),
                new Rotation3d(Units.degreesToRadians(180), -Units.degreesToRadians(30), Units.degreesToRadians(30)));

        //FR
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_2 = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.96), Units.inchesToMeters(-11.47), Units.inchesToMeters(8.32)),
                new Rotation3d(Units.degreesToRadians(180), -Units.degreesToRadians(30), Units.degreesToRadians(-30)));

        //BR
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_3 = new Transform3d(
                new Translation3d(Units.inchesToMeters(-10.96), Units.inchesToMeters(-11.47), Units.inchesToMeters(8.32)),
                new Rotation3d(0.0, -Units.degreesToRadians(30), Units.degreesToRadians(-150)));

        //BL
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_4 = new Transform3d(
                new Translation3d(Units.inchesToMeters(-10.96), Units.inchesToMeters(11.47), Units.inchesToMeters(8.32)),
                new Rotation3d(Units.degreesToRadians(180), -Units.degreesToRadians(30), Units.degreesToRadians(150)));

        // Calculated field length for 2024 game (used to circumvent "flipping tags" as
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
