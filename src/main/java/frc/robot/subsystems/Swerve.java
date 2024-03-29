package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.PathGenerationConstants;
import frc.robot.Constants.VisionConstants;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ShotCalculator;
import frc.lib.util.ShotCalculator.ShotData;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;
    private ChassisSpeeds chassisSpeeds;
    private PoseEstimatorSubsystem PoseEstimator;
    private SwerveDriveOdometry odo;
    private RotationOverride currentOverride = RotationOverride.NONE;

    private StructArrayPublisher<Pose3d> cameraFieldPoses = 
        NetworkTableInstance.getDefault()
        .getStructArrayTopic("Camera Field Positions", Pose3d.struct).publish();


    public Swerve() {

        /* Create NavX2 on MXP port */
        gyro = new AHRS(SerialPort.Port.kMXP);

        /* Zero gyro */
        zeroGyro();

        /* Create each swerve module */
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        /* Create gyro rotation and module position suppliers for PoseEstimatorSubsystem */
        Supplier<Rotation2d> rotSupplier = () -> getYaw();
        Supplier<SwerveModulePosition[]> modSupplier = () -> getModulePositions();

        /* Create PoseEstimatorSubsytem with two cameras */
        PoseEstimator = new PoseEstimatorSubsystem(rotSupplier, modSupplier, 
        //FL
            new PhotonRunnable("Arducam11", Constants.VisionConstants.ROBOT_TO_APRILTAG_CAMERA_1), 
        //FR
            new PhotonRunnable("Arducam_OV9281_USB_Camera", Constants.VisionConstants.ROBOT_TO_APRILTAG_CAMERA_2),
        //BR
            new PhotonRunnable("USB_Camera (1)", VisionConstants.ROBOT_TO_APRILTAG_CAMERA_3),
        //BL
            new PhotonRunnable("Arducam11B", VisionConstants.ROBOT_TO_APRILTAG_CAMERA_4)
        );

        odo = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        /* Initialize ChassisSpeeds */
        chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        /* Configure AutoBuilder and rotation override for PathPlanner paths */
        configureAutoBuilder();
    }

    /* Drive the swerve.
     * @param translation Translation2d containing x and y velocity in m/s
     * @param rotation rotational velocity in rad/s
     * @param fieldRelative enable/disable field relative driving
     * @param isOpenLoop enable/disable open loop driving
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getHeading()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation);
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(this.chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Drive the swerve.
     * @param output ChassisSpeeds object used to obtain module states
     */
    public void drive(ChassisSpeeds output) {
        this.chassisSpeeds = output;

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(this.chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Configure holonomic drivetrain in PathPlanner auto builder. */
    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, 
            this::getCurrentChassisSpeeds, this::drive, 
            new HolonomicPathFollowerConfig(
            Constants.Swerve.maxSpeed,
            Constants.Swerve.drivebaseRadius,
            new ReplanningConfig(true, false)
            ),
            () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
        );

        // May need to flip this depending on alliance. testing needed

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    /* Method used to obtain rotation target override for PPHolonomicDriveController.
     * @return optional Rotation2d object representing rotation override. Empty if none.
     */
    private Optional<Rotation2d> getRotationTargetOverride() {
        switch (currentOverride) {
            case NONE:
                return Optional.empty();
            case SPEAKER_AUTO_AIM:
                Pose2d tempPose = new Pose2d(0, 0, getShotData().goalHeading());
                // tempPose = AllianceFlipUtil.flipPose(tempPose);
                return Optional.of(tempPose.getRotation());
            default:
                return Optional.empty();
        }
    }

    public void setRotationTargetOverride(RotationOverride setting) {
        currentOverride = setting;
    }

    /* Method used by SwerveControllerCommand in dirtbikerx template code. 
     * @param desiredStates SwerveModuleState array to set each module to
    */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

    }

    /* @return whether or not a valid AprilTag has been seen since last robot disable. */
    public boolean getTagSeenSinceLastDisable() {
        return PoseEstimator.getTagSeenSinceLastDisable();
    }

    /* @return whether or not all Photonvision cameras are conneted to NT */
    public boolean allCamerasEnabled() {
        return PoseEstimator.allCamerasEnabled();
    }

    /* @return translational speed m/s (scalar) */
    public double getTranslationalSpeed() {
        return Math.hypot(getCurrentChassisSpeeds().vxMetersPerSecond, 
        getCurrentChassisSpeeds().vyMetersPerSecond);
    }

    /* @return rotational speed rad/s (scalar) */
    public double getRotationalSpeed() {
        return Math.abs(getCurrentChassisSpeeds().omegaRadiansPerSecond);
    }

    /* Generate a pathfinding command using PP AutoBuilder. 
     * Pose is automatically flipped according to alliance.
     * @param pose pose to pathfind
     * @param goalEndVelocity velocity in m/s at the end of the path
     * @param rotationDelayDistance how many seconds to wait before rotating
     * @return pathfinding command
     */
    public Command goToPose(Pose2d pose, double goalEndVelocity, double rotationDelayDistance) {
        Pose2d targetPose = pose;

        PathConstraints constraints = new PathConstraints(Constants.AutoConstants.velocityConstraint, Constants.AutoConstants.accelerationConstraint, 
        Constants.AutoConstants.angularVelocityConstraint, Constants.AutoConstants.angularAccelerationConstraint);
        
        return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, goalEndVelocity, rotationDelayDistance);
    }

    public Command pathfindToPathThenRun(String pathName) {
        PathConstraints constraints = new PathConstraints(Constants.AutoConstants.velocityConstraint, Constants.AutoConstants.accelerationConstraint, 
        Constants.AutoConstants.angularVelocityConstraint, Constants.AutoConstants.angularAccelerationConstraint);

        return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints, 0);
    }

    /* Gets robot pose (influenced by vision data). 
     * @return estimated pose
    */
    public Pose2d getPose() {
        return PoseEstimator.getCurrentPose(); 
    }

    public Pose2d flippedButNotRotationPose(Pose2d poseToFlip) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(
                VisionConstants.FIELD_LENGTH_METERS-poseToFlip.getX(), 
                poseToFlip.getY()), 
                Rotation2d.fromRotations(0.5)
                .rotateBy(poseToFlip.getRotation()));
        }
        return poseToFlip;
    }

    /* Resets robot pose.
     * @param pose to set robot to
     */
    public void resetPose(Pose2d pose) {
        PoseEstimator.setCurrentPose(pose);
    }

    /* Obtains ShotData for speaker.
     * @return auto aim shot data based on alliance-based speaker pose and current field relative pose/velocity
     */
    public ShotData getShotData() {
        return ShotCalculator.calculate(
          getAllianceSpeakerPos(),
          getPose().getTranslation(),
          // Obtain field relative chassis speeds
          new Translation2d(
            getCurrentChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos()
            - getCurrentChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
            getCurrentChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos()
            + getCurrentChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin()
        ));
    }

    public boolean isInClimbZone() {
        // since climb zones are mirrored
        Pose2d flippedPose = new Pose2d(new Translation2d(
            VisionConstants.FIELD_LENGTH_METERS-getPose().getX(), 
            getPose().getY()), 
            Rotation2d.fromRotations(0.5)
            .rotateBy(getPose().getRotation().unaryMinus()));

        return 
            // Within x bounds of climb area
            (
            (AllianceFlipUtil.flipPose(getPose()).getX() < 6.3 && AllianceFlipUtil.flipPose(getPose()).getX() > 2.7)
            &&
            // bounded by equations that define climb area
            (AllianceFlipUtil.flipPose(getPose()).getY() < 0.54 * AllianceFlipUtil.flipPose(getPose()).getX() + 3
            && AllianceFlipUtil.flipPose(getPose()).getY() > -0.56 * AllianceFlipUtil.flipPose(getPose()).getX() + 5.31)
            )
            
            ||
            
            (
            (AllianceFlipUtil.flipPose(flippedPose).getX() < 6.3 && AllianceFlipUtil.flipPose(flippedPose).getX() > 2.7)
            &&
            // bounded by equations that define climb area
            (AllianceFlipUtil.flipPose(flippedPose).getY() < 0.54 * AllianceFlipUtil.flipPose(flippedPose).getX() + 3
            && AllianceFlipUtil.flipPose(flippedPose).getY() > -0.56 * AllianceFlipUtil.flipPose(flippedPose).getX() + 5.31)
            );
    }

    /* @return SwerveModuleState array for each module */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /* @return SwerveModulePosition array for each module */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /* @return current ChassisSpeeds */
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return this.chassisSpeeds;
    }

    /* Zeros gyro (also clears angle adjustment) */
    public void zeroGyro(){
        setGyroAngleAdjustment(0);
        gyro.zeroYaw();
    }

    /* Set gyro angle adjustment */
    public void setGyroAngleAdjustment(double angleDegrees) {
        gyro.setAngleAdjustment(angleDegrees);
    }

    /* Use gyro angle adjustment to reset to odometry pose */
    public void setYawToOdometryPose() {
        gyro.zeroYaw();
        setGyroAngleAdjustment(
            (
                AllianceFlipUtil.flipPose(getPose())
                    .getRotation().getDegrees()
                -getYaw().getDegrees()
            ) % 360.0
        );
    }

    /* @return continuous Rotation2d object representing gyro yaw */
    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    /* @return field relative left camera Pose3d */
    public Pose3d getFrontLeftCameraFieldPosition() {
        Pose3d robotPose = new Pose3d(PoseEstimator.getCurrentPose());
        return robotPose.transformBy(VisionConstants.ROBOT_TO_APRILTAG_CAMERA_1);
    }

    /* @return field relative right camera Pose3d */
    public Pose3d getFrontRightCameraFieldPosition() {
        Pose3d robotPose = new Pose3d(PoseEstimator.getCurrentPose());
        return robotPose.transformBy(VisionConstants.ROBOT_TO_APRILTAG_CAMERA_2);
    }

    public Pose3d getBackRightCameraFieldPosition() {
        Pose3d robotPose = new Pose3d(PoseEstimator.getCurrentPose());
        return robotPose.transformBy(VisionConstants.ROBOT_TO_APRILTAG_CAMERA_3);
    }

    public Pose3d getBackLeftCameraFieldPosition() {
        Pose3d robotPose = new Pose3d(PoseEstimator.getCurrentPose());
        return robotPose.transformBy(VisionConstants.ROBOT_TO_APRILTAG_CAMERA_4);
    }

    /* @return alliance-based speaker position (Translation2d) */
    public Translation2d getAllianceSpeakerPos() {
        return AllianceFlipUtil.flipPose(new Pose2d(PathGenerationConstants.speakerTranslation, new Rotation2d()))
            .getTranslation();
    }

    /* Resets modules to absolute encoder forwards position. */
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void resetHeading() {
        odo.resetPosition(getYaw(), getModulePositions(), flippedButNotRotationPose(getPose()));
    }

    public void zeroHeading() {
        odo.resetPosition(getYaw(), getModulePositions(), flippedButNotRotationPose(new Pose2d(getPose().getTranslation(), new Rotation2d())));

    }

    public Rotation2d getHeading() {
        return odo.getPoseMeters().getRotation();
    }

    @Override
    public void periodic(){

        odo.update(getYaw(), getModulePositions());

        SmartDashboard.putString("Pose", "(" + getPose().getX()
        + ", " + getPose().getY() + "), " + getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro Rotation", getYaw().getDegrees());
        SmartDashboard.putNumber("Heading Angle", getHeading().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }

        SmartDashboard.putNumber("Drivetrain Translational Speed (m/s)", getTranslationalSpeed());
        SmartDashboard.putNumber("Drivetrain Rotational Speed (rad/s)", getRotationalSpeed());
        SmartDashboard.putNumber("Distance to Speaker (m)", getShotData().effectiveRobotToSpeakerDist());

        cameraFieldPoses.set(new Pose3d[] {getFrontLeftCameraFieldPosition(), getFrontRightCameraFieldPosition(), getBackRightCameraFieldPosition(), getBackLeftCameraFieldPosition()});
    }

    public enum RotationOverride {
        NONE, SPEAKER_AUTO_AIM;
    }
}