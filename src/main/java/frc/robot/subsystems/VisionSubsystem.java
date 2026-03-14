// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
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
 *
 * <p>Jitter fixes applied: - Removed duplicate getLatestResult() being added to
 * getAllUnreadResults() list - Per-camera rate limiting (front/rear no longer share a single fuse
 * timestamp) - Staleness guard: results older than 300ms are rejected - Rotation std devs heavily
 * increased to defer heading to gyro instead of vision
 *
 * <p>Bump crossing strategy (2026 field): The center bump causes wheel spin-up which corrupts
 * odometry. While the robot is in the bump zone, vision measurements are suppressed entirely and
 * the pose estimator relies on odometry only. Once the robot exits the bump zone and wheel speeds
 * stabilize, valid AprilTag readings are fused back in to correct any odometry drift accumulated
 * during the crossing. This mirrors the approach used by secretcitywildbots: "we use odometry
 * tracking and when we get valid position readings from the april tags we fuse it with our odometry
 * tracking."
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

  // Last estimated pose & timestamp
  private Optional<Pose2d> lastEstimatedPose = Optional.empty();
  private double lastEstimatedTimestamp = 0.0;

  // Track last accepted vision measurement for logging
  private boolean lastMeasurementAccepted = false;
  private int frontCameraTagCount = 0;
  private int rearCameraTagCount = 0;
  private double frontCameraAvgDistance = 0.0;
  private double rearCameraAvgDistance = 0.0;

  // Per-camera rate limiting — prevents front and rear from suppressing each other
  private double lastFrontFuseTime = 0.0;
  private double lastRearFuseTime = 0.0;

  // Maximum age of a vision result before it is discarded (seconds)
  private static final double MAX_RESULT_STALENESS_SEC = 0.3;

  // ── Bump zone suppression ─────────────────────────────────────────────────
  //
  // 2026 REBUILT field has 4 BUMPs — 2 per alliance side, flanking each HUB.
  // Per the game manual: each BUMP is 44.4 in (1.128 m) deep, centered 158.6 in
  // (4.028 m) from its alliance wall. Field total length = 651.2 in (16.54 m).
  //
  // WPILib coords: origin at Blue alliance wall, +X toward Red wall.
  //
  //   Blue-side bumps center X:  158.6 in  = 4.028 m  from Blue wall
  //   Red-side  bumps center X:  651.2 - 158.6 in = 492.6 in = 12.512 m from Blue wall
  //
  // BUMP_HALF_DEPTH = (44.4 in / 2) = 22.2 in = 0.564 m  (hard field dimension)
  // ROBOT_HALF_LEN  = add ~half your robot length so suppression starts before
  //                   the front wheel hits the bump and ends after the rear wheel clears.
  //                   Tune this on carpet — 0.25 m is a safe starting point.
  //
  // POST_BUMP_SETTLE_SEC — extra wait after exiting before re-enabling vision.
  //                        Wheels need a moment to stop spinning.
  //
  // Set BUMP_SUPPRESSION_ENABLED = false to disable entirely (e.g. during testing).
  private static final boolean BUMP_SUPPRESSION_ENABLED = true;

  // Half the physical bump depth (22.2 in = 0.564 m) + robot half-length buffer.
  // Increase ROBOT_HALF_LEN if wheels are still slipping when vision re-enables.
  private static final double BUMP_HALF_DEPTH = 0.564; // meters — fixed field dimension
  private static final double ROBOT_HALF_LEN = 0.25; // meters — TUNE to your robot
  private static final double BUMP_GUARD = BUMP_HALF_DEPTH + ROBOT_HALF_LEN;

  private static final double POST_BUMP_SETTLE_SEC = 0.25; // seconds — TUNE to your drivetrain

  // Center X positions (meters) of each bump pair, in WPILib field coordinates.
  // Blue-side: 158.6 in from Blue wall = 4.028 m
  // Red-side:  651.2 - 158.6 in       = 12.512 m
  private static final double[] BUMP_CENTER_XS = {4.028, 12.512};

  /** Timestamp when the robot last left any bump zone. -1 = never entered. */
  private double bumpExitTimestamp = -1.0;

  /** True while the robot's X position overlaps any bump zone. */
  private boolean inBumpZone = false;

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
            new Translation3d(Units.inchesToMeters(-1.25), 0, Units.inchesToMeters(25)),
            new Rotation3d(
                0, // roll
                Units.degreesToRadians(-20),
                0));

    robotToRearCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11), Units.inchesToMeters(0), Units.inchesToMeters(14.75)),
            new Rotation3d(0, 0, Units.degreesToRadians(-180)));

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
      System.err.println("[Vision] fieldLayout null -> poseEstimator disabled");
      throw new IllegalStateException(
          "[VisionSubsystem] AprilTagFieldLayout required for pose estimator");
    }

    // Simulation setup
    setupSimIfNeeded();
  }

  // ── Bump zone helpers ─────────────────────────────────────────────────────

  /**
   * Updates bump zone tracking state based on the robot's current field X position. Called once per
   * periodic() before processCamera() so both cameras see the same state.
   */
  private void updateBumpZoneState() {
    if (!BUMP_SUPPRESSION_ENABLED) return;

    double robotX = drivebase.getPose().getX();

    // Check all 4 bump centers (2 blue-side, 2 red-side share the same X position
    // since bumps are symmetric; we check both in case auto crosses the far side)
    boolean nowInZone = false;
    for (double bumpX : BUMP_CENTER_XS) {
      if (Math.abs(robotX - bumpX) < BUMP_GUARD) {
        nowInZone = true;
        break;
      }
    }

    // Rising edge — robot just entered a bump zone
    if (nowInZone && !inBumpZone) {
      Logger.recordOutput("Vision/Bump/EnteredZone", true);
    }

    // Falling edge — robot just exited a bump zone; start settle timer
    if (!nowInZone && inBumpZone) {
      bumpExitTimestamp = Timer.getFPGATimestamp();
      Logger.recordOutput("Vision/Bump/ExitedZone", true);
    }

    inBumpZone = nowInZone;
  }

  /**
   * Returns true if vision measurements should be suppressed right now.
   *
   * <p>Suppression is active while the robot is inside the bump zone AND for a short settle window
   * after it exits (wheel speeds need time to stabilize before odometry is trustworthy again and
   * before we want vision to fuse back in).
   */
  private boolean isBumpSuppressed() {
    if (!BUMP_SUPPRESSION_ENABLED) return false;
    if (DriverStation.isDisabled()) return false;
    if (inBumpZone) return true;
    if (bumpExitTimestamp < 0) return false;
    return (Timer.getFPGATimestamp() - bumpExitTimestamp) < POST_BUMP_SETTLE_SEC;
  }

  // ── Existing helpers (unchanged) ─────────────────────────────────────────

  private void setupSimIfNeeded() {
    if (!RobotBase.isSimulation()) {
      simEnabled = false;
      return;
    }

    simEnabled = true;

    visionSim = new VisionSystemSim("photonvision_sim");

    SimCameraProperties simProps = new SimCameraProperties();
    simProps.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    simProps.setFPS(60);
    simProps.setAvgLatencyMs(25);

    cameraSim = new PhotonCameraSim(camera, simProps);
    PhotonCameraSim rearCameraSim = new PhotonCameraSim(rearCamera, simProps);

    visionSim.addCamera(cameraSim, robotToCamera);
    visionSim.addCamera(rearCameraSim, robotToRearCamera);

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

    if (cam == camera) {
      frontCameraTagCount = tagCount;
      frontCameraAvgDistance = avgDistance;
    } else {
      rearCameraTagCount = tagCount;
      rearCameraAvgDistance = avgDistance;
    }

    if (tagCount == 0) return false;

    // Reject high-ambiguity single-tag estimates (ambiguity >= 0.2 is unreliable)
    if (tagCount == 1) {
      double ambiguity = est.targetsUsed.get(0).getPoseAmbiguity();
      if (ambiguity > 0.2 || ambiguity < 0) return false;
    }

    if (avgDistance > 6.0) return false;

    if (cam == rearCamera && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > 2.5)
      return false;

    return true;
  }

  /**
   * Computes dynamic standard deviations for the vision measurement.
   *
   * <p>Rotation std devs are intentionally very high so the gyro dominates heading. Vision is
   * primarily used for XY drift correction. Closer measurements from more tags get lower XY std
   * devs (higher trust).
   */
  private Matrix<N3, N1> computeStdDevs(int tagCount, double avgDistance) {
    // Multi-tag is much more accurate — trust it heavily for XY correction
    double baseXY = (tagCount >= 2) ? 0.05 : 0.15;
    double distScale = 1.0 + (avgDistance * avgDistance * 0.05);
    return VecBuilder.fill(baseXY * distScale, baseXY * distScale, 4.0);
  }

  private void processCamera(PhotonCamera cam, PhotonPoseEstimator estimator) {
    String cameraName = (cam == camera) ? "Front" : "Rear";

    // Fix: getAllUnreadResults() already contains the latest result.
    // Adding getLatestResult() on top caused duplicate processing and conflicting pose pushes.
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();

    // Use last element of unread results (or fall back to latest) for logging only
    PhotonPipelineResult latestResult =
        results.isEmpty() ? cam.getLatestResult() : results.get(results.size() - 1);

    Logger.recordOutput("Vision/" + cameraName + "/Connected", cam.isConnected());
    Logger.recordOutput("Vision/" + cameraName + "/HasTargets", latestResult.hasTargets());
    Logger.recordOutput("Vision/" + cameraName + "/TargetCount", latestResult.getTargets().size());

    if (latestResult.hasTargets()) {
      int[] tagIds =
          latestResult.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
      Logger.recordOutput("Vision/" + cameraName + "/VisibleTagIDs", tagIds);
    }

    double now = Timer.getFPGATimestamp();

    // Fix: use a per-camera fuse time so front and rear cameras don't suppress each other
    double lastFuseTime = (cam == camera) ? lastFrontFuseTime : lastRearFuseTime;

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> maybeEst = estimator.update(result);
      maybeEst.ifPresent(
          est -> {
            // Fix: staleness guard — discard results that are too old to be useful
            if (now - est.timestampSeconds > MAX_RESULT_STALENESS_SEC) {
              Logger.recordOutput("Vision/" + cameraName + "/RejectionReason", "Stale");
              return;
            }

            Pose2d est2d = est.estimatedPose.toPose2d();

            lastEstimatedPose = Optional.of(est2d);
            lastEstimatedTimestamp = est.timestampSeconds;

            // ── Bump suppression ────────────────────────────────────────────
            // While crossing the center bump (or settling just after), wheel spin-up
            // corrupts odometry. We trust odometry alone through the crossing and only
            // fuse AprilTag readings once wheel speeds have stabilised on the other side.
            if (isBumpSuppressed()) {
              lastMeasurementAccepted = false;
              Logger.recordOutput("Vision/" + cameraName + "/RejectionReason", "BumpZone");
              return;
            }

            // Fix: per-camera rate limit check instead of shared global
            if (now - lastFuseTime < VisionConstants.MIN_VISION_FUSE_PERIOD) {
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

            int tagCount = est.targetsUsed.size();
            double avgDistance =
                est.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(999.0);

            Matrix<N3, N1> stdDevs = computeStdDevs(tagCount, avgDistance);

            if (cam == camera) lastFrontFuseTime = now;
            else lastRearFuseTime = now;

            // Hard reset on high-confidence multi-tag close measurements —
            // cuts through wheel slip instantly instead of slowly blending.
            // NOTE: this hard-reset path is intentionally left active outside the bump
            // zone so that the first good multi-tag reading after the crossing snaps
            // the pose back to truth quickly.
            drivebase.addVisionMeasurement(
                est2d,
                est.timestampSeconds,
                stdDevs.get(0, 0),
                stdDevs.get(1, 0),
                stdDevs.get(2, 0));
            lastMeasurementAccepted = true;

            Logger.recordOutput("Vision/" + cameraName + "/AcceptedPose", est2d);
            Logger.recordOutput("Vision/" + cameraName + "/RejectionReason", "None");
            Logger.recordOutput("Vision/" + cameraName + "/StdDevXY", stdDevs.get(0, 0));
            Logger.recordOutput("Vision/" + cameraName + "/StdDevRot", stdDevs.get(2, 0));

            Logger.recordOutput("Vision/HasPose", true);
            Logger.recordOutput("Vision/PoseX", est2d.getX());
            Logger.recordOutput("Vision/PoseY", est2d.getY());
            Logger.recordOutput("Vision/PoseRotDeg", est2d.getRotation().getDegrees());
          });
    }

    Logger.recordOutput(
        "Vision/" + cameraName + "/TagCount",
        cam == camera ? frontCameraTagCount : rearCameraTagCount);
    Logger.recordOutput(
        "Vision/" + cameraName + "/AvgDistanceMeters",
        cam == camera ? frontCameraAvgDistance : rearCameraAvgDistance);
  }

  @Override
  public void periodic() {
    if (simEnabled && visionSim != null) {
      Pose2d pose2d = drivebase.getPose();
      if (pose2d != null) visionSim.update(pose2d);
    }

    // At the top of periodic(), after the sim update
    Pose2d currentPose = drivebase.getPose();
    SmartDashboard.putNumber("Robot/X", currentPose.getX());
    SmartDashboard.putNumber("Robot/Y", currentPose.getY());
    SmartDashboard.putNumber("Robot/HeadingDeg", currentPose.getRotation().getDegrees());

    Logger.recordOutput("Vision/isAimed", isAimed());

    // Update bump zone state once per loop so both cameras share identical suppression logic
    updateBumpZoneState();
    Logger.recordOutput("Vision/Bump/InZone", inBumpZone);
    Logger.recordOutput("Vision/Bump/Suppressed", isBumpSuppressed());
    Logger.recordOutput("Vision/Bump/RobotX", drivebase.getPose().getX());

    processCamera(camera, frontPoseEstimator);
    processCamera(rearCamera, rearPoseEstimator);

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

    Optional<Translation2d> scoringCenter = getScoringCenter();
    Logger.recordOutput("Vision/HasScoringCenter", scoringCenter.isPresent());
    if (scoringCenter.isPresent()) {
      Logger.recordOutput("Vision/ScoringCenterX", scoringCenter.get().getX());
      Logger.recordOutput("Vision/ScoringCenterY", scoringCenter.get().getY());
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Logger.recordOutput("Vision/Alliance", alliance.get().toString());
    }

    Logger.recordOutput("Vision/LastFrontFuseTime", lastFrontFuseTime);
    Logger.recordOutput("Vision/LastRearFuseTime", lastRearFuseTime);
    Logger.recordOutput(
        "Vision/TimeSinceLastFrontFuse", Timer.getFPGATimestamp() - lastFrontFuseTime);
    Logger.recordOutput(
        "Vision/TimeSinceLastRearFuse", Timer.getFPGATimestamp() - lastRearFuseTime);
  }

  // Returns the last vision-estimated Pose2d (field-relative) if available
  public Optional<Pose2d> getEstimatedPose() {
    return lastEstimatedPose;
  }

  // Returns the timestamp (seconds) of the last estimate, or 0.0 if none.
  public double getEstimatedPoseTimestamp() {
    return lastEstimatedPose.isPresent() ? lastEstimatedTimestamp : 0.0;
  }

  public Optional<Translation2d> getScoringCenter() {
    if (fieldLayout == null) return Optional.empty();

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    Set<Integer> redScoringTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    Set<Integer> blueScoringTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    Set<Integer> scoringTagIDs = (alliance == Alliance.Blue) ? blueScoringTags : redScoringTags;

    double sumX = 0.0, sumY = 0.0;
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

  /**
   * Returns the distance (meters) from the front camera (shooter-side) to the fuel goal center.
   * Used by LaunchCalculator so lookup table distances are measured from the actual launch point.
   */
  public Optional<Double> getDistanceToGoal() {
    Optional<Translation2d> maybeCenter = getScoringCenter();
    if (maybeCenter.isEmpty()) return Optional.empty();

    Pose2d robotPose = drivebase.getPose();

    // Project the front camera's field-relative position (shooter side)
    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();
    double camX = robotToCamera.getX();
    double camY = robotToCamera.getY();

    Translation2d shooterFieldPos =
        robotPose
            .getTranslation()
            .plus(new Translation2d(camX * cos - camY * sin, camX * sin + camY * cos));

    return Optional.of(shooterFieldPos.getDistance(maybeCenter.get()));
  }

  public Command rotateToAllianceTagWhileDriving(
      DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = drivebase.getPose();
              Optional<Translation2d> maybeCenter = getScoringCenter();

              double vx = vxSupplier.getAsDouble();
              double vy = vySupplier.getAsDouble();

              if (maybeCenter.isEmpty()) {
                drivebase.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, robotPose.getRotation()));
                Logger.recordOutput("Vision/Alignment/Active", false);
                return;
              }

              Translation2d center = maybeCenter.get();
              Translation2d toCenter = center.minus(robotPose.getTranslation());

              // Front of robot (shooter) faces the goal — no heading flip needed
              Rotation2d desiredHeading =
                  new Rotation2d(Math.atan2(toCenter.getY(), toCenter.getX()));

              double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

              double strafeComp = vy * 0.75;
              if (Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01) {
                strafeComp = 0.0;
              }

              double omega = (VisionConstants.ROTATION_KP * headingError + strafeComp) * 1.8;

              if (Math.abs(Math.toDegrees(headingError)) > VisionConstants.ANGLE_TOLERANCE_DEGREES
                  && Math.abs(omega) < VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC) {
                omega = Math.copySign(VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
              }

              omega =
                  Math.max(
                      -VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                      Math.min(VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

              drivebase.drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));

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

    if (maybeCenter.isEmpty()) return false;

    Translation2d center = maybeCenter.get();
    Translation2d toCenter = center.minus(robotPose.getTranslation());

    // Front of robot is the shooter — desired heading points directly at the goal
    Rotation2d desiredHeading = new Rotation2d(Math.atan2(toCenter.getY(), toCenter.getX()));

    double headingError = desiredHeading.minus(robotPose.getRotation()).getDegrees();
    return Math.abs(headingError) < VisionConstants.ANGLE_TOLERANCE_DEGREES;
  }

  // ── Shuttle shot helpers ──────────────────────────────────────────────────
  // WPILib standard: origin = blue wall, +X toward red wall, field length = 16.54 m.
  private static final double BLUE_WALL_X = 0.0;
  private static final double RED_WALL_X = 16.54;

  /** Distance (meters) from the robot to its own alliance end wall. */
  public Optional<Double> getDistanceToAllianceWall() {
    Pose2d robotPose = drivebase.getPose();
    if (robotPose == null) return Optional.empty();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    double wallX = (alliance == Alliance.Blue) ? BLUE_WALL_X : RED_WALL_X;
    return Optional.of(Math.abs(robotPose.getTranslation().getX() - wallX));
  }

  /**
   * Desired heading to face the alliance wall for a shuttle shot. Blue faces 180° (-X), Red faces
   * 0° (+X).
   */
  public Optional<Rotation2d> getShuttleHeading() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    return Optional.of(
        alliance == Alliance.Blue ? Rotation2d.fromDegrees(180.0) : Rotation2d.fromDegrees(0.0));
  }

  /** True when the robot is aimed at the alliance wall within ANGLE_TOLERANCE_DEGREES. */
  public boolean isAimedForShuttle() {
    Optional<Rotation2d> maybeHeading = getShuttleHeading();
    if (maybeHeading.isEmpty()) return false;
    double headingError = maybeHeading.get().minus(drivebase.getPose().getRotation()).getDegrees();
    return Math.abs(headingError) < VisionConstants.ANGLE_TOLERANCE_DEGREES;
  }

  /**
   * Rotates toward the alliance wall while allowing driver translation — mirrors
   * rotateToAllianceTagWhileDriving.
   */
  public Command rotateToShuttleHeadingWhileDriving(
      DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
    return this.run(
            () -> {
              Pose2d robotPose = drivebase.getPose();
              Optional<Rotation2d> maybeHeading = getShuttleHeading();
              double vx = vxSupplier.getAsDouble();
              double vy = vySupplier.getAsDouble();

              if (maybeHeading.isEmpty()) {
                drivebase.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, robotPose.getRotation()));
                return;
              }

              double headingError = maybeHeading.get().minus(robotPose.getRotation()).getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

              double strafeComp = (Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01) ? 0.0 : vy * 0.75;
              double omega = (VisionConstants.ROTATION_KP * headingError + strafeComp) * 1.8;

              if (Math.abs(Math.toDegrees(headingError)) > VisionConstants.ANGLE_TOLERANCE_DEGREES
                  && Math.abs(omega) < VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC) {
                omega = Math.copySign(VisionConstants.MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
              }

              omega =
                  Math.max(
                      -VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                      Math.min(VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

              drivebase.drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));

              Logger.recordOutput("Vision/Shuttle/HeadingErrorDeg", Math.toDegrees(headingError));
              Logger.recordOutput("Vision/Shuttle/OmegaRadPerSec", omega);
              Logger.recordOutput("Vision/Shuttle/Active", true);
            })
        .finallyDo(
            () -> {
              drivebase.drive(new ChassisSpeeds(0, 0, 0));
              Logger.recordOutput("Vision/Shuttle/Active", false);
            });
  }

  public Command rotateToHeadingWhileDriving(
      DoubleSupplier vxSupplier, DoubleSupplier vySupplier, Rotation2d targetHeading) {
    return this.run(
            () -> {
              Pose2d robotPose = drivebase.getPose();

              // Scale joystick -1..1 to actual m/s
              double vx = vxSupplier.getAsDouble() * DrivebaseConstants.MAX_SPEED;
              double vy = vySupplier.getAsDouble() * DrivebaseConstants.MAX_SPEED;

              double headingError = targetHeading.minus(robotPose.getRotation()).getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

              double omega = (VisionConstants.ROTATION_KP * headingError) * 1.8;

              drivebase.drive(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));

              Logger.recordOutput("Vision/BumpAlign/TargetHeadingDeg", targetHeading.getDegrees());
              Logger.recordOutput("Vision/BumpAlign/HeadingErrorDeg", Math.toDegrees(headingError));
              Logger.recordOutput("Vision/BumpAlign/OmegaRadPerSec", omega);
              Logger.recordOutput("Vision/BumpAlign/Active", true);
            })
        .finallyDo(
            () -> {
              drivebase.drive(new ChassisSpeeds(0, 0, 0));
              Logger.recordOutput("Vision/BumpAlign/Active", false);
            });
  }

  // ── Climb helpers ─────────────────────────────────────────────────────────

  // Tag IDs for the climb zone entry on each side
  private static final int[] RED_CLIMB_TAG_IDS = {15, 16};
  private static final int[] BLUE_CLIMB_TAG_IDS = {31, 32};

  // How far in front of the tag midpoint the robot should stop (meters)
  // Tune this so the robot is fully inside the climb zone
  private static final double CLIMB_APPROACH_STOP_METERS = 0.55;

  // Speed limits (m/s)
  private static final double CLIMB_APPROACH_SPEED = 0.6; // moving toward zone
  private static final double CLIMB_CREEP_SPEED = 0.25; // final entry
  private static final double CLIMB_CREEP_THRESHOLD = 0.6; // switch to creep within this distance

  // Tolerances
  private static final double CLIMB_POSITION_TOLERANCE = 0.06; // meters — close enough to stop
  private static final double CLIMB_HEADING_TOLERANCE_DEG = 3.0;

  /**
   * Returns the target Pose2d to drive to for climbing — midpoint between the two climb tags,
   * offset inward by CLIMB_APPROACH_STOP_METERS.
   */
  public Optional<Pose2d> getClimbTargetPose() {
    if (fieldLayout == null) return Optional.empty();

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int[] tagIds = (alliance == Alliance.Red) ? RED_CLIMB_TAG_IDS : BLUE_CLIMB_TAG_IDS;

    Optional<Pose2d> tag1 = fieldLayout.getTagPose(tagIds[0]).map(p -> p.toPose2d());
    Optional<Pose2d> tag2 = fieldLayout.getTagPose(tagIds[1]).map(p -> p.toPose2d());

    if (tag1.isEmpty() || tag2.isEmpty()) return Optional.empty();

    // True center between the two uprights (not tag faces)
    Translation2d mid = tag1.get().getTranslation().plus(tag2.get().getTranslation()).div(2.0);

    // Use average tag rotation for a fixed perpendicular heading — not robot-relative
    double avgHeadingRad =
        (tag1.get().getRotation().getRadians() + tag2.get().getRotation().getRadians()) / 2.0;
    Rotation2d facingIn = new Rotation2d(avgHeadingRad).plus(Rotation2d.fromDegrees(180.0));
    // Lateral correction: shift stop point toward the opening center
    // The opening is 24.00" wide; tags sit on uprights offset ~11.38" from wall
    // Shift perpendicular to the entry axis to center on the gap
    double lateralOffsetMeters = Units.inchesToMeters(0); // tune if still off-center
    Translation2d lateralShift =
        new Translation2d(
            -facingIn.getSin() * lateralOffsetMeters, facingIn.getCos() * lateralOffsetMeters);

    // Drive to just inside the opening
    Translation2d stopPoint =
        mid.plus(
                new Translation2d(
                    CLIMB_APPROACH_STOP_METERS * facingIn.getCos(),
                    CLIMB_APPROACH_STOP_METERS * facingIn.getSin()))
            .plus(lateralShift);

    return Optional.of(new Pose2d(stopPoint, facingIn));
  }

  public Command alignAndClimb() {
    final Pose2d[] capturedTarget = {null};

    return this.runOnce(() -> getClimbTargetPose().ifPresent(t -> capturedTarget[0] = t))
        .andThen(
            this.run(
                () -> {
                  if (capturedTarget[0] == null) {
                    drivebase.drive(new ChassisSpeeds(0, 0, 0));
                    return;
                  }

                  Pose2d robotPose = drivebase.getPose();
                  Pose2d target = capturedTarget[0];
                  Translation2d toTarget =
                      target.getTranslation().minus(robotPose.getTranslation());
                  double distance = toTarget.getNorm();

                  // ── Rotation control ──────────────────────────────────────────────
                  double headingError =
                      target.getRotation().minus(robotPose.getRotation()).getRadians();
                  headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
                  double omega = VisionConstants.ROTATION_KP * headingError * 1.8;
                  omega =
                      Math.max(
                          -VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                          Math.min(VisionConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

                  // ── Translation control ───────────────────────────────────────────
                  boolean headingSettled =
                      Math.abs(Math.toDegrees(headingError)) < CLIMB_HEADING_TOLERANCE_DEG;
                  double speed =
                      (distance < CLIMB_CREEP_THRESHOLD) ? CLIMB_CREEP_SPEED : CLIMB_APPROACH_SPEED;

                  // Drive purely along the locked heading axis — eliminates diagonal drift
                  double forwardSpeed = headingSettled ? speed : 0.0;
                  double vx = forwardSpeed * target.getRotation().getCos();
                  double vy = forwardSpeed * target.getRotation().getSin();

                  drivebase.drive(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          vx, vy, omega, robotPose.getRotation()));

                  Logger.recordOutput("Climb/DistanceToTarget", distance);
                  Logger.recordOutput("Climb/HeadingErrorDeg", Math.toDegrees(headingError));
                  Logger.recordOutput("Climb/Speed", speed);
                  Logger.recordOutput("Climb/Active", true);
                }))
        .until(
            () -> {
              if (capturedTarget[0] == null) return false;
              return capturedTarget[0]
                      .getTranslation()
                      .minus(drivebase.getPose().getTranslation())
                      .getNorm()
                  < CLIMB_POSITION_TOLERANCE;
            })
        .finallyDo(
            () -> {
              drivebase.drive(new ChassisSpeeds(0, 0, 0));
              Logger.recordOutput("Climb/Active", false);
            });
  }
}
