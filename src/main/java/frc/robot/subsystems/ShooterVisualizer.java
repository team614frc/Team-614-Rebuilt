package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Landmarks;
import frc.util.FuelSim;
import java.util.function.Supplier;

/**
 * Shooter visualizer for simulation.
 *
 * <p>Simulates a real hooded shooter by redistributing kinetic energy: - Low hood → flatter, faster
 * shots - High hood → slower forward speed, higher arc
 *
 * <p>Gravity + FuelSim produce the curve naturally.
 */
public class ShooterVisualizer {

  // Geometry (Meters)
  private static final double SHOOTER_HEIGHT = 0.705;
  private static final double SHOOTER_FORWARD_OFFSET = -0.3; // Shooter is behind robot center
  private static final double SHOOTER_LATERAL_SPACING = 0.13;

  private static final int MAX_FUEL_CAPACITY = 54;

  // Hood Angle Mapping
  private static final double MIN_ANGLE_DEG = 42.0;
  private static final double MAX_ANGLE_DEG = 65.0;

  // Shooter Speed Limits
  // ADJUSTED: Increased speeds to compensate for shooter being 0.3m behind center
  private static final double MIN_SPEED = 6.5; // close shots (was 6.0)
  private static final double MAX_SPEED = 10.0; // far shots (was 9.0)

  // Visual Randomness
  private static final double VELOCITY_REL_VARIATION = 0.05;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Angle> hoodAngleSupplier;
  private final Supplier<LinearVelocity> shooterVelocitySupplier;

  private int fuelStored = 0;

  public ShooterVisualizer(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Angle> hoodAngleSupplier,
      Supplier<LinearVelocity> shooterVelocitySupplier) {

    this.robotPoseSupplier = robotPoseSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.shooterVelocitySupplier = shooterVelocitySupplier;

    SmartDashboard.putNumber("Sim/Fuel Stored", fuelStored);
  }

  // Intake simulation
  public void simIntake() {
    if (fuelStored < MAX_FUEL_CAPACITY) {
      fuelStored++;
      SmartDashboard.putNumber("Sim/Fuel Stored", fuelStored);
    }
  }

  public boolean simCanIntake() {
    return fuelStored < MAX_FUEL_CAPACITY;
  }

  // Shooting simulation
  public void launchFuel() {
    if (fuelStored <= 0) return;

    int desired = Math.random() < 0.8 ? 3 : 2;
    int toLaunch = Math.min(desired, fuelStored);

    Pose2d pose = robotPoseSupplier.get();
    Angle hoodAngle = hoodAngleSupplier.get();
    LinearVelocity shooterVel = shooterVelocitySupplier.get();

    // Hood Normalization
    double hoodDeg = mapHoodAngleToDegrees(hoodAngle);
    double hoodT = MathUtil.inverseInterpolate(MIN_ANGLE_DEG, MAX_ANGLE_DEG, hoodDeg);

    SmartDashboard.putNumber("Sim/Hood_mapped_deg", hoodDeg);
    SmartDashboard.putNumber("Sim/Hood_t", hoodT);

    // Distance to High Goal
    Translation2d goalPos = Landmarks.hubPosition();
    double distance = pose.getTranslation().getDistance(goalPos);

    // ADJUSTED: Add offset distance since shooter is 0.3m behind robot center
    // This gives us the true distance from shooter to target
    double effectiveDistance = distance + Math.abs(SHOOTER_FORWARD_OFFSET);
    double distanceFactor = Math.min(effectiveDistance / 8.5, 1.0); // 0 = close, 1 = far

    SmartDashboard.putNumber("Sim/Distance_m", distance);
    SmartDashboard.putNumber("Sim/Effective_Distance_m", effectiveDistance);

    // Base shooter speed (simulating real power adjustment)
    double baseSpeed = MathUtil.clamp(shooterVel.in(MetersPerSecond), MIN_SPEED, MAX_SPEED);
    baseSpeed = MathUtil.interpolate(MIN_SPEED, MAX_SPEED, distanceFactor);

    SmartDashboard.putNumber("Sim/BaseSpeed_mps", baseSpeed);

    // Blend horizontal and vertical scaling based on distance & hood
    double blendFactor = 0.5 * distanceFactor + 0.5 * hoodT;

    double verticalScale = MathUtil.interpolate(1.0, 3.0, blendFactor); // close=low, far=high
    double horizontalScale =
        MathUtil.interpolate(0.5, 0.2, blendFactor); // close=more horiz, far=less

    SmartDashboard.putNumber("Sim/VertScale", verticalScale);
    SmartDashboard.putNumber("Sim/HorizScale", horizontalScale);

    double angleRad = Math.toRadians(hoodDeg);

    for (int i = 0; i < toLaunch; i++) {
      fuelStored--;

      int barrel = i % 3;
      double lateralOffset =
          (barrel == 0) ? -SHOOTER_LATERAL_SPACING : (barrel == 2 ? SHOOTER_LATERAL_SPACING : 0.0);

      double velMult = 1.0 + (Math.random() - 0.5) * 2.0 * VELOCITY_REL_VARIATION;

      double speed = baseSpeed * velMult;

      // Compute velocities
      double baseHoriz = speed * Math.cos(angleRad) * horizontalScale;
      double baseVert = speed * Math.sin(angleRad) * verticalScale;

      Rotation2d heading = pose.getRotation();
      Translation2d horizVec = new Translation2d(baseHoriz, 0.0).rotateBy(heading);

      double spawnX =
          pose.getX()
              + SHOOTER_FORWARD_OFFSET * heading.getCos()
              - lateralOffset * heading.getSin();
      double spawnY =
          pose.getY()
              + SHOOTER_FORWARD_OFFSET * heading.getSin()
              + lateralOffset * heading.getCos();

      Translation3d spawnPos = new Translation3d(spawnX, spawnY, SHOOTER_HEIGHT);
      Translation3d spawnVel = new Translation3d(horizVec.getX(), horizVec.getY(), baseVert);

      FuelSim.getInstance().spawnFuel(spawnPos, spawnVel);
    }

    SmartDashboard.putNumber("Sim/Fuel Stored", fuelStored);
  }

  // Hood mapping
  private double mapHoodAngleToDegrees(Angle hoodAngle) {
    double maybeDeg = hoodAngle.in(Degrees);

    if (maybeDeg < 10.0) {
      double unit = hoodAngle.in(Rotations);
      double t = MathUtil.clamp(unit, 0.0, 1.0);
      return MIN_ANGLE_DEG + t * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
    }

    return MathUtil.clamp(maybeDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
  }

  // Utility
  public int getFuelCount() {
    return fuelStored;
  }

  public void setFuelCount(int count) {
    fuelStored = Math.max(0, Math.min(count, MAX_FUEL_CAPACITY));
    SmartDashboard.putNumber("Sim/Fuel Stored", fuelStored);
  }

  public void resetFuel() {
    setFuelCount(0);
  }

  public void periodic() {
    SmartDashboard.putNumber("Score/Blue Hub", FuelSim.Hub.BLUE_HUB.getScore());
    SmartDashboard.putNumber("Score/Red Hub", FuelSim.Hub.RED_HUB.getScore());
    SmartDashboard.putNumber(
        "Score/Total", FuelSim.Hub.BLUE_HUB.getScore() + FuelSim.Hub.RED_HUB.getScore());
    SmartDashboard.putNumber("Sim/Fuel Stored", fuelStored);
  }
}
