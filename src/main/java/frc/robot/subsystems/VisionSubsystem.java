// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * VisionSubsystem
 *
 * <p>Keeps alignment command intact - Uses PhotonPoseEstimator per Photon docs/javadocs:
 * PhotonPoseEstimator(AprilTagFieldLayout, PoseStrategy, Transform3d) - Loops over
 * camera.getAllUnreadResults() and calls poseEstimator.update(result) - Stores latest Estimated
 * pose and timestamp; attempts to apply it to your SwerveSubsystem by reflection
 */
public class VisionSubsystem extends SubsystemBase {

  private static final String CAMERA_NAME = "spatulas_eye";
  private static final String REAR_CAMERA_NAME = "spatulas_eye_back";

  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
  private final PhotonCamera rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

  private PhotonCameraSim cameraSim = null;
  private VisionSystemSim visionSim = null;
  private boolean simEnabled = false;
  private AprilTagFieldLayout fieldLayout = null;

  private final SwerveSubsystem drivebase;

  // Fields for PhotonPoseEstimator usage
  private final Transform3d robotToCamera;
  private final Transform3d robotToRearCamera;
  private final PhotonPoseEstimator frontPoseEstimator;
  private final PhotonPoseEstimator rearPoseEstimator;
  // May be null if fieldLayout is unavailable

  // Last estimated pose & timestamp
  private Optional<Pose2d> lastEstimatedPose = Optional.empty();
  private double lastEstimatedTimestamp = 0.0;

  public VisionSubsystem(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;

    // Load field layout
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    } catch (Exception ex) {
      System.err.println("[Vision] Could not load AprilTagFieldLayout: " + ex.getMessage());
      fieldLayout = null;
    }

    /**
     * Robot to camera transform offset
     *
     * <p>Don't make the same mistake I did, we use FREEDOM units here. When setting an offset in
     * rotation, the unit is in Radians. When setting an offset in length/width, the unit is in
     * Meters. Reference off the WPILib Coordinate System for positive/negative values for the
     * offset.
     */
    robotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0.5), Units.inchesToMeters(-13), Units.inchesToMeters(11)),
            new Rotation3d(0, Units.degreesToRadians(-30), Math.PI));

    robotToRearCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(24), Units.inchesToMeters(0), Units.inchesToMeters(11)),
            new Rotation3d(0, Units.degreesToRadians(15), 0));

    // Create photon pose estimator
    if (fieldLayout != null) {
      frontPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

      frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      rearPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToRearCamera);

      rearPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } else {
      // If the layout is not available, don't create estimator
      // Set it to be null so the robot doesn't explode
      System.err.println("[Vision] fieldLayout null -> poseEstimator disabled");
      throw new IllegalStateException(
          "[VisionSubsystem] AprilTagFieldLayout required for pose estimator");
    }

    // Simulation setup
    setupSimIfNeeded();
  }

  private void setupSimIfNeeded() {
    if (!RobotBase.isSimulation()) {
      simEnabled = false;
      return;
    }

    simEnabled = true;

    // Create vision system FIRST
    visionSim = new VisionSystemSim("photonvision_sim");

    // Sim camera properties
    SimCameraProperties simProps = new SimCameraProperties();
    simProps.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    simProps.setFPS(60);
    simProps.setAvgLatencyMs(25);

    cameraSim = new PhotonCameraSim(camera, simProps);
    PhotonCameraSim rearCameraSim = new PhotonCameraSim(rearCamera, simProps);

    // Attach cameras
    visionSim.addCamera(cameraSim, robotToCamera);
    visionSim.addCamera(rearCameraSim, robotToRearCamera);

    // Add AprilTags
    if (fieldLayout != null) {
      visionSim.addAprilTags(fieldLayout);
    }

    cameraSim.enableDrawWireframe(true);
    SmartDashboard.putData("Vision Field", visionSim.getDebugField());
  }

  private boolean isVisionMeasurementTrusted(EstimatedRobotPose est, PhotonCamera cam) {

    int tagCount = est.targetsUsed.size();

    double avgDistance =
        est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(999.0);

    // Reject garbage
    if (tagCount == 0) return false;

    // Reject far poses
    if (avgDistance > 4.5) return false;

    // Reject single-tag far shots
    if (tagCount == 1 && avgDistance > 2.5) return false;

    // Reject rear camera while spinning fast
    if (cam == rearCamera && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > 2.5) {
      return false;
    }

    return true;
  }

  private void processCamera(PhotonCamera cam, PhotonPoseEstimator estimator) {

    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    results.add(cam.getLatestResult());

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> maybeEst = estimator.update(result);
      maybeEst.ifPresent(
          est -> {
            Pose2d est2d = est.estimatedPose.toPose2d();

            lastEstimatedPose = Optional.of(est2d);
            lastEstimatedTimestamp = est.timestampSeconds;

            double now = Timer.getFPGATimestamp();

            if (now - VisionConstants.lastVisionFuseTime < VisionConstants.MIN_VISION_FUSE_PERIOD) {
              return;
            }

            if (!isVisionMeasurementTrusted(est, cam)) {
              return;
            }

            VisionConstants.lastVisionFuseTime = now;
            drivebase.addVisionMeasurement(est2d, est.timestampSeconds);

            SmartDashboard.putBoolean("Vision/HasPose", true);
            SmartDashboard.putNumber("Vision/PoseX", est2d.getX());
            SmartDashboard.putNumber("Vision/PoseY", est2d.getY());
            SmartDashboard.putNumber("Vision/PoseRotDeg", est2d.getRotation().getDegrees());
          });
    }
  }

  @Override
  public void periodic() {
    // Update simulation if needed
    if (simEnabled && visionSim != null) {
      Pose2d pose2d = drivebase.getPose();
      if (pose2d != null) visionSim.update(pose2d);
    }

    SmartDashboard.putBoolean("Vision/isAimed", isAimed());

    // Process latest vision results
    // Include latest if none unread. It's deprecated, but it's a great fallback
    processCamera(camera, frontPoseEstimator);
    processCamera(rearCamera, rearPoseEstimator);
  }

  // Returns the last vision-estimated Pose2d (field-relative) if available
  public Optional<Pose2d> getEstimatedPose() {
    return lastEstimatedPose;
  }

  // Returns the timestamp (seconds) of the last estimate, or 0.0 if none.
  public double getEstimatedPoseTimestamp() {
    return lastEstimatedPose.isPresent() ? lastEstimatedTimestamp : 0.0;
  }

  // Computes the geometric center of the scoring area using alliance-specific AprilTags
  public Optional<Translation2d> getScoringCenter() {
    if (fieldLayout == null) return Optional.empty();

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    // Red HUB side
    Set<Integer> redScoringTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    // Blue HUB side
    Set<Integer> blueScoringTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    Set<Integer> scoringTagIDs = (alliance == Alliance.Blue) ? blueScoringTags : redScoringTags;

    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;

    for (AprilTag tag : fieldLayout.getTags()) {
      if (!scoringTagIDs.contains(tag.ID)) continue;

      Translation2d pos = tag.pose.toPose2d().getTranslation();
      sumX += pos.getX();
      sumY += pos.getY();
      count++;
    }

    if (count == 0) return Optional.empty();

    return Optional.of(new Translation2d(sumX / count, sumY / count));
  }

  public Command rotateToAllianceTagWhileDriving(
      DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = drivebase.getPose();
              Optional<Translation2d> maybeCenter = getScoringCenter();

              double vx = vxSupplier.getAsDouble();
              double vy = vySupplier.getAsDouble();

              // If we can't compute center, just drive normally
              if (maybeCenter.isEmpty()) {
                drivebase.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, robotPose.getRotation()));
                return;
              }

              Translation2d center = maybeCenter.get();

              // Vector from robot to the scoring center
              Translation2d toCenter = center.minus(robotPose.getTranslation());

              // Shooter faces the center
              Rotation2d desiredHeading =
                  new Rotation2d(Math.atan2(toCenter.getY(), toCenter.getX()));

              double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

              // Strafing compensation (keeps shots centered while moving)
              double strafeComp = vy * 0.75;
              if (Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01) {
                strafeComp = 0.0;
              }

              double omega = (VisionConstants.ROTATION_KP * headingError + strafeComp) * 1.8;

              // Minimum rotation snap
              if (Math.abs(Math.toDegrees(headingError)) > VisionConstants.ANGLE_TOLERANCE_DEGREES
                  && Math.abs(omega) < VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC) {
                omega = Math.copySign(VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
              }

              omega =
                  Math.max(
                      -VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                      Math.min(VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

              // The robot has the taste of freedom now
              drivebase.drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));

              // Debug output
              SmartDashboard.putNumber(
                  "Vision/ScoringCenterErrorDeg", Math.toDegrees(headingError));
              SmartDashboard.putNumber("Vision/Omega", omega);
            })
        .finallyDo(
            () -> {
              drivebase.drive(new ChassisSpeeds(0, 0, 0));
            });
  }

  public boolean isAimed() {
    Pose2d robotPose = drivebase.getPose();
    Optional<Translation2d> maybeCenter = getScoringCenter();

    if (maybeCenter.isEmpty()) {
      return false; // Can't aim if we don't know where the hub is
    }

    Translation2d center = maybeCenter.get();
    Translation2d toCenter = center.minus(robotPose.getTranslation());

    Rotation2d desiredHeading = new Rotation2d(Math.atan2(toCenter.getY(), toCenter.getX()));

    double headingError = desiredHeading.minus(robotPose.getRotation()).getDegrees();
    return Math.abs(headingError) < 5.0; // tolerance in degrees
  }
}
