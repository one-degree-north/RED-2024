package frc.robot.subsystems;



import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;


/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimatorSubsystem extends SubsystemBase implements Logged {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */

  // TODO: Figure out what stddevs we should be using
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 1.8);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  private boolean tagSeenSinceLastDisable = false;
  
  private PhotonRunnable[] cameras;
  private Notifier[] cameraNotifiers;

  // remove camera field2ds if monologue implementation works
  private Field2d[] cameraField2ds;

  private ArrayList<Pose2d> arrayListVisionPoses = new ArrayList<>();

  // private final PhotonRunnable photonEstimator = new PhotonRunnable("Arducam_OV9281_USB_Camera");
  
  // TODO: Figure out if notifier is necessary
  // private final Notifier photonNotifier = new Notifier(photonEstimator);

  public PoseEstimatorSubsystem(
      Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier, PhotonRunnable... cameras) {
    
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;
    this.cameras = cameras;
    // Initialize the array of Notifiers
    this.cameraNotifiers = new Notifier[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      this.cameraNotifiers[i] = new Notifier(cameras[i]);
    }

    // Initialize the array of Field2ds
    this.cameraField2ds = new Field2d[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      this.cameraField2ds[i] = new Field2d();
      SmartDashboard.putData("Camera " + i, cameraField2ds[i]);
    }

    poseEstimator =  new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    // Start PhotonVision threads
    for (int i = 0; i < cameraNotifiers.length; i++) {
      cameraNotifiers[i].setName("PhotonRunnable " + i);
      cameraNotifiers[i].startPeriodic(0.02);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

    ArrayList<Pose2d> tempArrayList = new ArrayList<>();

    // TODO: Make this create multiple Field2ds for each camera
    for (int i=0; i < cameras.length; i++) {
      var visionPose = cameras[i].grabLatestEstimatedPose();
      if (visionPose != null) {
        // New pose from vision
        if (!DriverStation.isEnabled()) tagSeenSinceLastDisable = true;
        
        var pose2d = visionPose.estimatedPose.toPose2d();

        Vector<N3> dynamicVisionMeasurementStdDevs;

        if (cameras[i].getTrackedVisionTargets().length > 0) {
          Pose3d closestTarget = cameras[i].getTrackedVisionTargets()[0];
          for (Pose3d target : cameras[i].getTrackedVisionTargets()) {
            if (
              target.toPose2d().getTranslation()
              .getDistance(pose2d.getTranslation())
              <
              closestTarget.toPose2d().getTranslation()
              .getDistance(pose2d.getTranslation())
            ) 
            {
              closestTarget = target;
            }
          }
          // scale with distance to closest target and number of targets
          double scalingFactor = 
            closestTarget.toPose2d().getTranslation().getDistance(pose2d.getTranslation())
          * (1.0/cameras[i].getTrackedVisionTargets().length);

          dynamicVisionMeasurementStdDevs = visionMeasurementStdDevs.times(scalingFactor);
          
        } else {
          dynamicVisionMeasurementStdDevs = visionMeasurementStdDevs;
        }
        
        poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds, dynamicVisionMeasurementStdDevs);

        // Set the pose on the dashboard
        cameraField2ds[i].setRobotPose(pose2d);
        tempArrayList.add(pose2d);
      } else {
        // Jank workaround
        cameraField2ds[i].setRobotPose(new Pose2d(1000, 1000, new Rotation2d()));
      }
    }

    // Reset tagSeenSinceLastDisable when a robot is disabled
    if (tagSeenSinceLastDisable && DriverStation.isEnabled())
      tagSeenSinceLastDisable = false;

    // Set the pose on the dashboard
    var dashboardPose = poseEstimator.getEstimatedPosition();

    SmartDashboard.putString("Pose", getFormattedPose());

    field2d.setRobotPose(dashboardPose);
    SmartDashboard.putData("Pose Estimator Field2d", field2d);

    arrayListVisionPoses = tempArrayList;

  }

  @Log
  public Pose2d[] getArrayOfVisionPoses() {
    return arrayListVisionPoses.toArray(new Pose2d[arrayListVisionPoses.size()]);
  }

  public PhotonRunnable[] getCameras() {
    return cameras;
  }

  public boolean allCamerasEnabled() {
    for (PhotonRunnable camera : cameras) {
      if (!camera.isConnected()) return false;
    }
    return true;
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public boolean getTagSeenSinceLastDisable() {
    return tagSeenSinceLastDisable;
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}