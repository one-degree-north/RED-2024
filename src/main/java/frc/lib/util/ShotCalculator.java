package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Code is from Mechanical Advantage's util classes
 */
public class ShotCalculator {
  public record ShotData(
      double effectiveRobotToSpeakerDist,
      double radialFeedforward, // ff value due to radial velocity of robot to speaker
      Rotation2d goalHeading,
      double clampedArmAngle) {} // heading of robot to match tangential velocity

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

    // TODO: what does this do
    // Ig this is the estimated time of the note in the air
    // later on this will be a function of the distance
    double shotTime = 1.05;

    // Add robot velocity to raw shot speed
    double rawDistToGoal = robot.getDistance(speaker);
    double shotSpeed = rawDistToGoal / shotTime + radialComponent;
    if (shotSpeed <= 0.0) shotSpeed = 0.0;
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    // Rotation is accounted for when inverting pose
    Rotation2d goalHeading =
        GeomUtil.inverse(GeomUtil.toPose2d(robot)).transformBy(GeomUtil.toTransform2d(speaker)).getTranslation().getAngle();
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent)).plus(Rotation2d.fromRotations(0.5));
    double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);

    // This will be replaced with a formula to return arm angle in radians based on distance in meters
    /* IN ROTATIONS */
    double armAngle = effectiveDist;

    // Potentially have this as output
    /* IN ROTATIONS */
    double clampedArmAngle = MathUtil.clamp(armAngle, 
        Constants.ElevatarmConstants.armReverseSoftLimit, 
        Constants.ElevatarmConstants.armForwardSoftLimit);

    // Use radial component of velocity for ff value
    return new ShotData(effectiveDist, radialComponent, goalHeading, clampedArmAngle);
  }
}