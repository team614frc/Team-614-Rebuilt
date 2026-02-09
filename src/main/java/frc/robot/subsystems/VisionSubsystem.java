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
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

  // Track last accepted vision measurement for logging
  private boolean lastMeasurementAccepted = false;
  private int frontCameraTagCount = 0;
  private int rearCameraTagCount = 0;
  private double frontCameraAvgDistance = 0.0;
  private double rearCameraAvgDistance = 0.0;

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
    // Field2d is a Sendable; record it to SmartDashboard instead of Logger which doesn't accept
    // Field2d
    SmartDashboard.putData("Vision Field", visionSim.getDebugField());
  }

  private boolean isVisionMeasurementTrusted(EstimatedRobotPose est, PhotonCamera cam) {

    int tagCount = est.targetsUsed.size();

    double avgDistance =
        est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(999.0);

    // Store for logging
    if (cam == camera) {
      frontCameraTagCount = tagCount;
      frontCameraAvgDistance = avgDistance;
    } else {
      rearCameraTagCount = tagCount;
      rearCameraAvgDistance = avgDistance;
    }

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
    String cameraName = (cam == camera) ? "Front" : "Rear";

    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    results.add(cam.getLatestResult());

    // Log camera connection and latest result
    PhotonPipelineResult latestResult = cam.getLatestResult();
    Logger.recordOutput("Vision/" + cameraName + "/Connected", cam.isConnected());
    Logger.recordOutput("Vision/" + cameraName + "/HasTargets", latestResult.hasTargets());
    Logger.recordOutput("Vision/" + cameraName + "/TargetCount", latestResult.getTargets().size());

    // Log visible tag IDs
    if (latestResult.hasTargets()) {
      int[] tagIds =
          latestResult.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
      Logger.recordOutput("Vision/" + cameraName + "/VisibleTagIDs", tagIds);
    }

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
              lastMeasurementAccepted = false;
              Logger.recordOutput("Vision/" + cameraName + "/RejectionReason", "RateLimited");
              return;
            }

            if (!isVisionMeasurementTrusted(est, cam)) {
              lastMeasurementAccepted = false;
              Logger.recordOutput(
                  "Vision/" + cameraName + "/RejectionReason", "UntrustedMeasurement");
              return;
            }

            VisionConstants.lastVisionFuseTime = now;
            drivebase.addVisionMeasurement(est2d, est.timestampSeconds);
            lastMeasurementAccepted = true;

            // Log accepted measurement
            Logger.recordOutput("Vision/" + cameraName + "/AcceptedPose", est2d);
            Logger.recordOutput("Vision/" + cameraName + "/RejectionReason", "None");

            Logger.recordOutput("Vision/HasPose", true);
            Logger.recordOutput("Vision/PoseX", est2d.getX());
            Logger.recordOutput("Vision/PoseY", est2d.getY());
            Logger.recordOutput("Vision/PoseRotDeg", est2d.getRotation().getDegrees());
          });
    }

    // Log measurement quality metrics
    Logger.recordOutput(
        "Vision/" + cameraName + "/TagCount",
        cam == camera ? frontCameraTagCount : rearCameraTagCount);
    Logger.recordOutput(
        "Vision/" + cameraName + "/AvgDistanceMeters",
        cam == camera ? frontCameraAvgDistance : rearCameraAvgDistance);
  }

  @Override
  public void periodic() {
    // Update simulation if needed
    if (simEnabled && visionSim != null) {
      Pose2d pose2d = drivebase.getPose();
      if (pose2d != null) visionSim.update(pose2d);
    }

    Logger.recordOutput("Vision/isAimed", isAimed());

    // Process latest vision results
    processCamera(camera, frontPoseEstimator);
    processCamera(rearCamera, rearPoseEstimator);

    // Log overall vision state
    Logger.recordOutput("Vision/HasEstimatedPose", lastEstimatedPose.isPresent());
    Logger.recordOutput("Vision/LastMeasurementAccepted", lastMeasurementAccepted);
    Logger.recordOutput("Vision/IsAimed", isAimed());
    Logger.recordOutput("Vision/SimEnabled", simEnabled);

    if (lastEstimatedPose.isPresent()) {
      Pose2d pose = lastEstimatedPose.get();
      Logger.recordOutput("Vision/EstimatedPose", pose);
      Logger.recordOutput("Vision/EstimatedPoseX", pose.getX());
      Logger.recordOutput("Vision/EstimatedPoseY", pose.getY());
      Logger.recordOutput("Vision/EstimatedPoseRotationDeg", pose.getRotation().getDegrees());
      Logger.recordOutput("Vision/EstimatedPoseTimestamp", lastEstimatedTimestamp);
    }

    // Log scoring center calculation
    Optional<Translation2d> scoringCenter = getScoringCenter();
    Logger.recordOutput("Vision/HasScoringCenter", scoringCenter.isPresent());
    if (scoringCenter.isPresent()) {
      Logger.recordOutput("Vision/ScoringCenterX", scoringCenter.get().getX());
      Logger.recordOutput("Vision/ScoringCenterY", scoringCenter.get().getY());
    }

    // Log alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Logger.recordOutput("Vision/Alliance", alliance.get().toString());
    }

    // Log timing
    Logger.recordOutput("Vision/LastVisionFuseTime", VisionConstants.lastVisionFuseTime);
    Logger.recordOutput(
        "Vision/TimeSinceLastFuse", Timer.getFPGATimestamp() - VisionConstants.lastVisionFuseTime);
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
                Logger.recordOutput("Vision/Alignment/Active", false);
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
              Logger.recordOutput("Vision/ScoringCenterErrorDeg", Math.toDegrees(headingError));
              Logger.recordOutput("Vision/Omega", omega);

              // Log alignment details
              Logger.recordOutput("Vision/Alignment/Active", true);
              Logger.recordOutput("Vision/Alignment/HeadingErrorDeg", Math.toDegrees(headingError));
              Logger.recordOutput("Vision/Alignment/OmegaRadPerSec", omega);
              Logger.recordOutput("Vision/Alignment/StrafeCompensation", strafeComp);
              Logger.recordOutput(
                  "Vision/Alignment/DesiredHeadingDeg", desiredHeading.getDegrees());
              Logger.recordOutput("Vision/Alignment/DistanceToCenter", toCenter.getNorm());
            })
        .finallyDo(
            () -> {
              drivebase.drive(new ChassisSpeeds(0, 0, 0));
              Logger.recordOutput("Vision/Alignment/Active", false);
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
