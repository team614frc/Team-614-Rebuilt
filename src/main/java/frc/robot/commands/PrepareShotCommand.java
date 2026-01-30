package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class PrepareShotCommand extends Command {
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

  static {
    distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
    distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
    distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
  }

  private final Shooter shooter;
  private final Hood hood;
  private final Supplier<Pose2d> robotPoseSupplier;

  public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.hood = hood;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(shooter, hood);
  }

  public boolean isReadyToShoot() {
    boolean hoodReady = hood.isPositionWithinTolerance();
    boolean shooterReady;

    // In simulation, skip the velocity check since motors don't simulate
    if (RobotBase.isSimulation()) {
      shooterReady = true; // Always consider shooter ready in sim
      SmartDashboard.putBoolean("PrepareShot/Hood Ready", hoodReady);
      SmartDashboard.putBoolean("PrepareShot/Shooter Ready (sim)", shooterReady);
      SmartDashboard.putBoolean("PrepareShot/Ready to Shoot", hoodReady);
      System.out.println(
          "PrepareShot - Hood Ready: " + hoodReady + ", Ready to Shoot: " + hoodReady);
    } else {
      // On real robot, check both shooter velocity and hood position
      shooterReady = shooter.isVelocityWithinTolerance();
      SmartDashboard.putBoolean("PrepareShot/Hood Ready", hoodReady);
      SmartDashboard.putBoolean("PrepareShot/Shooter Ready", shooterReady);
      SmartDashboard.putBoolean("PrepareShot/Ready to Shoot", hoodReady && shooterReady);
    }

    return hoodReady && shooterReady;
  }

  private Distance getDistanceToHub() {
    final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
    final Translation2d hubPosition = Landmarks.hubPosition();
    return Meters.of(robotPosition.getDistance(hubPosition));
  }

  @Override
  public void execute() {
    final Distance distanceToHub = getDistanceToHub();
    final Shot shot = distanceToShotMap.get(distanceToHub);
    shooter.setRPM(shot.shooterRPM);
    hood.setPosition(shot.hoodPosition);
    SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    SmartDashboard.putNumber("PrepareShot/Target RPM", shot.shooterRPM);
    SmartDashboard.putNumber("PrepareShot/Target Hood Position", shot.hoodPosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    System.out.println("PrepareShot command ended - interrupted: " + interrupted);
  }

  public static class Shot {
    public final double shooterRPM;
    public final double hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      this.shooterRPM = shooterRPM;
      this.hoodPosition = hoodPosition;
    }
  }
}
