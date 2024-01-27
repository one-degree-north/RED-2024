package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private final Field2d visionPose;
  private Pose2d visionPoseToUpdate = null;

  public PhotonRunnable(String cameraName, Transform3d ROBOT_TO_APRILTAG_CAMERA) {
    this.photonCamera = new PhotonCamera(cameraName); 
    PhotonPoseEstimator photonPoseEstimator = null;
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (photonCamera != null) {
      photonPoseEstimator = new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, ROBOT_TO_APRILTAG_CAMERA);
    }
    this.photonPoseEstimator = photonPoseEstimator;
    this.visionPose = new Field2d();
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      var photonResults = photonCamera.getLatestResult();
      visionPoseToUpdate = null;

      // Consider targets underneath ambiguity threshold OR if there are multiple tags
      if (photonResults.hasTargets() 
          && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) 
      {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
            visionPoseToUpdate = estimatedPose.toPose2d();
          }
        });
      }
    }
    
    // Set Field2d vision pose
    visionPose.setRobotPose(visionPoseToUpdate);
  }

  public boolean isConnected() {
    return photonCamera.isConnected();
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
