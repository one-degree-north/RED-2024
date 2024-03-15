package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ElevatarmConstants;
import frc.robot.Constants.MechanismSetpointConstants;

/**
 * Code is from Mechanical Advantage's util classes
 * This does not technically properly account for velocity perfectly, but should work in practice at low robot velocities
 */
public class ShotCalculator {
  public record ShotData(
      double effectiveRobotToSpeakerDist,
      double radialFeedforward, // ff value due to radial velocity of robot to speaker
      Rotation2d goalHeading,
      double clampedArmAngle,
      boolean inYDistanceRange) {} // heading of robot to match tangential velocity

  /** In theory we will aim at different locations inside speaker */
  public static ShotData calculate(
      Translation2d speaker, Translation2d robot, Translation2d linearFieldVelocity) {
    // Calculate radial and tangential velocity from speaker
    Rotation2d speakerToRobotAngle = robot.minus(speaker).getAngle();
    Translation2d tangentialVelocity =
        linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
    // Positive when velocity is away from speaker
    double radialComponent = tangentialVelocity.getX();
    // Positive when traveling CCW about speaker
    double tangentialComponent = tangentialVelocity.getY();

    double rawDistToGoal = robot.getDistance(speaker);

    double shotTime = 0.5;

    // Add robot velocity to raw shot speed
    double shotSpeed = rawDistToGoal / shotTime - radialComponent;
    if (shotSpeed <= 0.0) shotSpeed = 0.0;
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    // Rotation is accounted for when inverting pose
    Rotation2d goalHeading =
        GeomUtil.inverse(GeomUtil.toPose2d(robot)).transformBy(GeomUtil.toTransform2d(speaker)).getTranslation().getAngle();
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent)).plus(Rotation2d.fromRotations(0.5));

    // (incorrectly) assume that shot time does not change while moving radially
    // double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);
    double effectiveDist = rawDistToGoal;

    // This will be replaced with a formula to return arm angle in rotations based on distance in meters
    /* IN ROTATIONS */
    // double armAngle = -0.0327 + 0.0493 * effectiveDist - 0.00417 * Math.pow(effectiveDist, 2);
    double armAngle = -0.1824 * Math.atan(-2.105 * effectiveDist + 0.7838) - 0.1321;

    // Potentially have this as output
    /* IN ROTATIONS */
    double clampedArmAngle = MathUtil.clamp(armAngle, 
        Constants.ElevatarmConstants.armReverseSoftLimit, 
        ElevatarmConstants.armForwardSoftLimit);
    
    boolean inYDistanceRange = Math.abs(robot.getY()-speaker.getY()) < MechanismSetpointConstants.yDistanceDifferenceToAutoScore;

    // Use radial component of velocity for ff value
    return new ShotData(effectiveDist, radialComponent, goalHeading, clampedArmAngle, inYDistanceRange);
  }
}