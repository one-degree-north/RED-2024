// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class AllianceFlipUtil {
    /* Flip pose according to alliance detected on Driverstation.
     * @param poseToFlip pose that may or may not be flipped
     * @return flipped pose if on red alliance, origianl pose if on blue alliance
     */
    public static Pose2d flipPose(Pose2d poseToFlip) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(
                VisionConstants.FIELD_LENGTH_METERS-poseToFlip.getX(), 
                poseToFlip.getY()), 
                poseToFlip.getRotation().rotateBy(Rotation2d.fromRotations(0.5)));
        }
        return poseToFlip;
    }
}
