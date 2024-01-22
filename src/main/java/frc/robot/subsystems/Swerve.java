package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;
    private ChassisSpeeds chassisSpeeds;
    private PoseEstimatorSubsystem PoseEstimator;


    public Swerve() {

        gyro = new AHRS(SerialPort.Port.kMXP);

        // Zero gyro after reset (shouldn't technically be needed)
        zeroGyro();

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

        Supplier<Rotation2d> rotSupplier = () -> getYaw();
        Supplier<SwerveModulePosition[]> modSupplier = () -> getModulePositions();

        PoseEstimator = new PoseEstimatorSubsystem(rotSupplier, modSupplier, 
            new PhotonRunnable("Arducam11", Constants.VisionConstants.APRILTAG_CAMERA_1_TO_ROBOT), 
            new PhotonRunnable("Arducam12", Constants.VisionConstants.APRILTAG_CAMERA_2_TO_ROBOT));

        chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        configureAutoBuilder();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
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
    
    public void drive(ChassisSpeeds output) {
        this.chassisSpeeds = output;

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(this.chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(this::getPhotonPose, this::resetPhotonPose, 
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
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

    }

    public boolean getTagSeenSinceLastDisable() {
        return PoseEstimator.getTagSeenSinceLastDisable();
    }

    public boolean allCamerasEnabled() {
        return PoseEstimator.allCamerasEnabled();
    }

    public double getSpeed() {
        return Math.hypot(getCurrentChassisSpeeds().vxMetersPerSecond, 
        getCurrentChassisSpeeds().vyMetersPerSecond);
    }

    public Rotation2d getHeading(Pose2d initial, Pose2d end) {
        double distanceX = end.getX()-initial.getX();
        double distanceY = end.getY()-initial.getY();
        return Rotation2d.fromRadians(
            Math.atan2(distanceY, distanceX));
    }


    // only testing on this generateonthefly method
    public Command goToPose(Pose2d pose, double goalEndVelocity, double rotationDelayDistance, boolean flipPose) {
        var alliance = DriverStation.getAlliance();
        Pose2d targetPose;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetPose = (!flipPose ? pose : flipAlliance(pose));
        } else {
            targetPose = pose;
        }

        PathConstraints constraints = new PathConstraints(Constants.AutoConstants.velocityConstraint, Constants.AutoConstants.accelerationConstraint, 
        Constants.AutoConstants.angularVelocityConstraint, Constants.AutoConstants.angularAccelerationConstraint);
        
        return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity, rotationDelayDistance);
    }

    public Pose2d flipAlliance(Pose2d poseToFlip) {
        return new Pose2d(new Translation2d(VisionConstants.FIELD_LENGTH_METERS-poseToFlip.getX(), poseToFlip.getY()), poseToFlip.getRotation().rotateBy(Rotation2d.fromRotations(0.5)));
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public Pose2d getPhotonPose() {
        return PoseEstimator.getCurrentPose();
    }

    public void resetPhotonPose(Pose2d pose) {
        PoseEstimator.setCurrentPose(pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return this.chassisSpeeds;
    }

    //TODO: make sure everything with gyro works as intended
    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){

        SmartDashboard.putString("getPhotonPose", "(" + getPhotonPose().getX()
        + ", " + getPhotonPose().getY() + "), " + getPhotonPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Gyro Rotation", getYaw().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
    }
}