package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Landmarks;
import frc.util.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
  private static final Distance SHOOTER_HEIGHT = Meters.of(0.705);
  private static final Distance SHOOTER_FORWARD_OFFSET =
      Meters.of(-0.3); // Shooter is behind robot center
  private static final Distance SHOOTER_LATERAL_SPACING = Meters.of(0.13);

  private static final int MAX_FUEL_CAPACITY = 54;

  // Hood Angle Mapping
  private static final Angle MIN_ANGLE = Degrees.of(42.0);
  private static final Angle MAX_ANGLE = Degrees.of(65.0);

  // Shooter Speed Limits
  // ADJUSTED: Increased speeds to compensate for shooter being 0.3m behind center
  private static final LinearVelocity MIN_SPEED = MetersPerSecond.of(6.5); // close shots (was 6.0)
  private static final LinearVelocity MAX_SPEED = MetersPerSecond.of(10.0); // far shots (was 9.0)

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

    Logger.recordOutput("Sim/Fuel Stored", fuelStored);
  }

  // Intake simulation
  public void simIntake() {
    if (fuelStored < MAX_FUEL_CAPACITY) {
      fuelStored++;
      Logger.recordOutput("Sim/Fuel Stored", fuelStored);
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
    double hoodT =
        MathUtil.inverseInterpolate(MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees), hoodDeg);

    Logger.recordOutput("Sim/Hood_mapped_deg", hoodDeg);
    Logger.recordOutput("Sim/Hood_t", hoodT);

    // Distance to High Goal
    Translation2d goalPos = Landmarks.hubPosition();
    double distance = pose.getTranslation().getDistance(goalPos);

    // ADJUSTED: Add offset distance since shooter is 0.3m behind robot center
    // This gives us the true distance from shooter to target
    double effectiveDistance = distance + Math.abs(SHOOTER_FORWARD_OFFSET.in(Meters));
    double distanceFactor = Math.min(effectiveDistance / 8.5, 1.0); // 0 = close, 1 = far

    Logger.recordOutput("Sim/Distance_m", distance);
    Logger.recordOutput("Sim/Effective_Distance_m", effectiveDistance);

    // Base shooter speed (simulating real power adjustment)
    double baseSpeed =
        MathUtil.clamp(
            shooterVel.in(MetersPerSecond),
            MIN_SPEED.in(MetersPerSecond),
            MAX_SPEED.in(MetersPerSecond));
    baseSpeed =
        MathUtil.interpolate(
            MIN_SPEED.in(MetersPerSecond), MAX_SPEED.in(MetersPerSecond), distanceFactor);

    Logger.recordOutput("Sim/BaseSpeed_mps", baseSpeed);

    // Blend horizontal and vertical scaling based on distance & hood
    double blendFactor = 0.5 * distanceFactor + 0.5 * hoodT;

    double verticalScale = MathUtil.interpolate(1.0, 3.0, blendFactor); // close=low, far=high
    double horizontalScale =
        MathUtil.interpolate(0.5, 0.2, blendFactor); // close=more horiz, far=less

    Logger.recordOutput("Sim/VertScale", verticalScale);
    Logger.recordOutput("Sim/HorizScale", horizontalScale);

    double angleRad = Math.toRadians(hoodDeg);

    for (int i = 0; i < toLaunch; i++) {
      fuelStored--;

      int barrel = i % 3;
      double lateralOffset =
          (barrel == 0)
              ? -SHOOTER_LATERAL_SPACING.in(Meters)
              : (barrel == 2 ? SHOOTER_LATERAL_SPACING.in(Meters) : 0.0);

      double velMult = 1.0 + (Math.random() - 0.5) * 2.0 * VELOCITY_REL_VARIATION;

      double speed = baseSpeed * velMult;

      // Compute velocities
      double baseHoriz = speed * Math.cos(angleRad) * horizontalScale;
      double baseVert = speed * Math.sin(angleRad) * verticalScale;

      Rotation2d heading = pose.getRotation();
      Translation2d horizVec = new Translation2d(baseHoriz, 0.0).rotateBy(heading);

      double spawnX =
          pose.getX()
              + SHOOTER_FORWARD_OFFSET.in(Meters) * heading.getCos()
              - lateralOffset * heading.getSin();
      double spawnY =
          pose.getY()
              + SHOOTER_FORWARD_OFFSET.in(Meters) * heading.getSin()
              + lateralOffset * heading.getCos();

      Translation3d spawnPos = new Translation3d(spawnX, spawnY, SHOOTER_HEIGHT.in(Meters));
      Translation3d spawnVel = new Translation3d(horizVec.getX(), horizVec.getY(), baseVert);

      FuelSim.getInstance().spawnFuel(spawnPos, spawnVel);
    }

    Logger.recordOutput("Sim/Fuel Stored", fuelStored);
  }

  // Hood mapping
  private double mapHoodAngleToDegrees(Angle hoodAngle) {
    double maybeDeg = hoodAngle.in(Degrees);

    if (maybeDeg < 10.0) {
      double unit = hoodAngle.in(Rotations);
      double t = MathUtil.clamp(unit, 0.0, 1.0);
      return MAX_ANGLE.minus(MIN_ANGLE).times(t).plus(MIN_ANGLE).in(Degrees);
    }

    return MathUtil.clamp(maybeDeg, MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees));
  }

  // Utility
  public int getFuelCount() {
    return fuelStored;
  }

  public void setFuelCount(int count) {
    fuelStored = Math.max(0, Math.min(count, MAX_FUEL_CAPACITY));
    Logger.recordOutput("Sim/Fuel Stored", fuelStored);
  }

  public void resetFuel() {
    setFuelCount(0);
  }

  public void periodic() {
    Logger.recordOutput("Score/Blue Hub", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("Score/Red Hub", FuelSim.Hub.RED_HUB.getScore());
    Logger.recordOutput(
        "Score/Total", FuelSim.Hub.BLUE_HUB.getScore() + FuelSim.Hub.RED_HUB.getScore());
    Logger.recordOutput("Sim/Fuel Stored", fuelStored);
  }
}
