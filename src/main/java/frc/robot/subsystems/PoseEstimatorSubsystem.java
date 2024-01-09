package frc.robot.subsystems;



import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

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
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final Field2d visionField2d = new Field2d();
  
  private PhotonRunnable[] cameras;
  private Notifier[] cameraNotifiers;

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

    // TODO: Make this create multiple Field2ds for each camera
    for (PhotonRunnable photonEstimator : cameras) {
      var visionPose = photonEstimator.grabLatestEstimatedPose();
      if (visionPose != null) {
        // New pose from vision
        var pose2d = visionPose.estimatedPose.toPose2d();
        
        poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);

        visionField2d.setRobotPose(pose2d);
        SmartDashboard.putData("Vision Field2d", visionField2d);
      }
    }

    // Set the pose on the dashboard
    var dashboardPose = poseEstimator.getEstimatedPosition();

    SmartDashboard.putString("Pose", getFormattedPose());

    field2d.setRobotPose(dashboardPose);
    SmartDashboard.putData("Pose Estimator Field2d", field2d);

  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
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
