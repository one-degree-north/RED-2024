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
    public static final class TeleopConstants {
        public static final double stickDeadband = 0.1;
        public static final double rateLimitXY = 15;
        public static final double rateLimitTheta = 15*Math.PI;

        // TODO: Tune PID
        public static final double headingkP = 0.02;
        public static final double headingkI = 0;
        public static final double headingkD = 0;
        public static final double headingMaxVelRadPerSec = AutoConstants.angularVelocityConstraint;
        public static final double headingMaxAccelRadPerSec = AutoConstants.angularAccelerationConstraint;
    }

    public static final class PathGenerationConstants {
        // This is always from the perspective of the driver (left and right flip depending on alliance)
        public static final Pose2d leftSpeakerScoringPose = new Pose2d(3.78, 7.01, new Rotation2d(0));
        public static final Pose2d middleSpeakerScoringPose = new Pose2d(3.91, 5.64, new Rotation2d(0));
        public static final Pose2d rightSpeakerScoringPose = new Pose2d(2.1, 4.23, new Rotation2d(0));
        public static final Pose2d ampScoringPose = new Pose2d();


        public static final Translation2d speakerPosition = 
            new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

        // This is always from the perspective of the driver (left and right flip depending on alliance)
        public static final Pose2d leftSourceIntakingPose = new Pose2d();
        public static final Pose2d middleSourceIntakingPose = new Pose2d();
        public static final Pose2d rightSourceIntakingPose = new Pose2d();

    }

    public static final class ElevatarmConstants {
        // Arm IDs
        public static final int armLeaderID = 0;
        public static final int armFollowerID = 0;
        public static final int armEncoderPort = 0;

        public static final double armAbsoluteEncoderOffset = 0.0;
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

        public static final double armGearRatio = 0.0;

        public static final int elevatorID = 0;
        public static final int elevatorEncoderPort = 0;
        public static final int elevatorLimitMinPort = 0;
        public static final int elevatorLimitMaxPort = 0;

        public static final double elevatorAbsoluteEncoderOffset = 0.0;
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

        public static final double elevatorAbsoluteSensorToMotorRatio = 0.0;
        public static final double elevatorEncoderRotationsPerDistance = 0.0;

        // Distance from pivot to center of fully retracted end effector
        public static final double mindistanceFromPivot = 0.0;
        public static final double maxDistanceFromPivot = 0.0;
    }

    public static final class ClimbConstants {
        public static final int leftClimbID = 0;
        public static final int rightClimbID = 0;
        
        public static final int leftEncoderPort = 0;
        public static final int rightEncoderPort = 0;

        // in meters
        public static final double leftAbsoluteEncoderOffset = 0;
        public static final double rightAbsoluteEncoderOffset = 0;

        public static final double absoluteSensorToMotorRatio = 0;
        public static final double encoderRotationsPerDistance = 0;

        public static final double positionkP = 0.0;
        public static final double positionkI = 0.0;
        public static final double positionkD = 0.0;
        public static final double positionkG = 0.0;
        public static final double positionkS = 0.0;
        public static final double positionkV = 0.0;
        public static final double positionkA = 0.0;
        // Meters per second
        public static final double cruiseVelocity = 0.0;
        public static final double acceleration = 0.0;

        public static final double velocitykP = 0.0;
        public static final double velocitykI = 0.0;
        public static final double velocitykD = 0.0;
        public static final double velocitykG = 0.0;
        public static final double velocitykS = 0.0;
        public static final double velocitykV = 0.0;
        public static final double velocitykA = 0.0;

        public static final double forwardSoftLimit = 0.0;
        public static final double reverseSoftLimit = 0.0;


    }

    // TODO: Tune slew rate limiter to driver's preferences (this is basically acceleration)

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Comment out if not using supported module
        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);
        /* Drivetrain Constants */
        // This must be tuned to specific robot.
        public static final double trackWidth = Units.inchesToMeters(17.75); 
        public static final double wheelBase = Units.inchesToMeters(17.75); 
        public static final double drivebaseRadius = Math.hypot(trackWidth, wheelBase)/2.0;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
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

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        // DISREGARD IF USING SUPPORTED MODULES
        // This must be tuned to specific robot. We can do this through Phoenix Tuner
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        //TODO: This must be tuned to specific robot. We can do this through Phoenix Tuner
        public static final double driveKP = 0.135; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        //kF depricated in Phoenix 6
        // public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // This must be tuned to specific robot. We can do this by locking the rotation gears and using SYSID.
        public static final double driveKS = (0.12698); 
        public static final double driveKV = (2.1248);
        public static final double driveKA = (0.14197); // TODO: Used to divide by 12 to convert

        /* Swerve Profiling Values */
        // This must be tuned to specific robot.
        /** Meters per Second */
        public static final double maxSpeed = 5.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 4*Math.PI; 

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
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.269);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.277);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.927);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }




    public static class VisionConstants {
        // These must be tuned to specific robot.

        // 3D Translation from AprilTag camera to center of robot
        //TODO: Get offset values
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(9), Units.inchesToMeters(8), Units.inchesToMeters(8.75)),
            new Rotation3d(0.0, -Units.degreesToRadians(30), -Units.degreesToRadians(45)));
        public static final Transform3d ROBOT_TO_APRILTAG_CAMERA_2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(-8), Units.inchesToMeters(8.75)),
            new Rotation3d(0.0, -Units.degreesToRadians(30), Units.degreesToRadians(135)));
        
        // Calculated field length for 2023 game (used to circumvent "flipping tags" as well as mirror coordinates for red/blue alliance)
        public static final double FIELD_LENGTH_METERS = 
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldLength();
        public static final double FIELD_WIDTH_METERS = 
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getFieldWidth();
    
        // Minimum target ambiguity. Targets with higher ambiguity will be discarded 
        //TODO: Test ambiguity
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
      }



      
    public static final class AutoConstants { 
        // TODO: These must be tuned to specific robot
        public static final double velocityConstraint = Swerve.maxSpeed;
        public static final double accelerationConstraint = Swerve.maxSpeed;
        public static final double angularVelocityConstraint = Swerve.maxAngularVelocity;
        public static final double angularAccelerationConstraint = Swerve.maxAngularVelocity*1.2;
    }
}
