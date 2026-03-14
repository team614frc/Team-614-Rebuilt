package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.util.AllianceShiftMonitor;
import frc.util.ShuttleCalculator;
import java.util.Set;
import java.util.function.DoubleSupplier;

public final class SubsystemCommands {
  private final SwerveSubsystem swerve;
  private final Intake intake;
  private final Floor floor;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Hood hood;
  private final Hanger hanger;
  private final VisionSubsystem vision;
  private final AllianceShiftMonitor shiftMonitor;

  private final DoubleSupplier forwardInput;
  private final DoubleSupplier leftInput;

  public SubsystemCommands(
      SwerveSubsystem swerve,
      Intake intake,
      Floor floor,
      Feeder feeder,
      Shooter shooter,
      Hood hood,
      Hanger hanger,
      VisionSubsystem vision,
      AllianceShiftMonitor shiftMonitor,
      DoubleSupplier forwardInput,
      DoubleSupplier leftInput) {
    this.swerve = swerve;
    this.intake = intake;
    this.floor = floor;
    this.feeder = feeder;
    this.shooter = shooter;
    this.hood = hood;
    this.hanger = hanger;
    this.vision = vision;
    this.shiftMonitor = shiftMonitor;
    this.forwardInput = forwardInput;
    this.leftInput = leftInput;
  }

  public SubsystemCommands(
      SwerveSubsystem swerve,
      Intake intake,
      Floor floor,
      Feeder feeder,
      Shooter shooter,
      Hood hood,
      Hanger hanger,
      VisionSubsystem vision,
      AllianceShiftMonitor shiftMonitor) {
    this(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        vision,
        shiftMonitor,
        () -> 0,
        () -> 0);
  }

  public Command aimAndShoot() {
    return Commands.defer(
        () -> {
          // Fresh instance every time defer runs (i.e. every time the button is pressed).
          // Creating it outside defer caused WPILib to mark it as "composed" on the first
          // press and crash with IllegalArgumentException on every subsequent press.
          final PrepareShotCommand prepareShotCommand =
              new PrepareShotCommand(shooter, hood, () -> swerve.getPose());

          return Commands.parallel(
              // Branch 1: rotate drivetrain toward goal
              vision.rotateToAllianceTagWhileDriving(forwardInput, leftInput),

              // Branch 2: spin up flywheel + set hood angle.
              // 0.20 s delay staggers power draw so drivetrain and shooter
              // don't both slam the PDH at the same time.
              Commands.waitSeconds(0.20).andThen(prepareShotCommand),

              // Branch 3: wait until aimed AND shooter/hood at setpoint AND hub is active,
              // THEN feed.
              Commands.sequence(
                  Commands.waitUntil(
                      () ->
                          vision.isAimed()
                              && prepareShotCommand.isReadyToShoot()
                              && shiftMonitor.isOurHubActive()),
                  feed()));
        },
        Set.of(swerve, vision, shooter, hood, feeder, floor, intake));
  }

  public Command shuttleFuel() {
    return Commands.defer(
        () -> {
          // Snapshot distance once when the command is composed
          double distanceMeters = vision.getDistanceToAllianceWall().orElse(7.0);
          double rpm = ShuttleCalculator.getShooterRPM(distanceMeters);
          double hoodPos = ShuttleCalculator.getHoodPosition(distanceMeters);

          return Commands.race(
              // Branch 1: rotate to face alliance wall while driver can still translate
              vision.rotateToShuttleHeadingWhileDriving(forwardInput, leftInput),

              // Branch 2: spin up shooter + set hood, then wait until aimed, then feed
              Commands.sequence(
                  hood.positionCommand(hoodPos).alongWith(shooter.spinUpCommand(rpm)),
                  Commands.waitUntil(
                      () -> vision.isAimedForShuttle() && shooter.isVelocityWithinTolerance()),
                  feed()));
        },
        Set.of(swerve, vision, shooter, hood, feeder, floor, intake));
  }

  public Command shootManually() {
    return Commands.sequence(
        shooter.dashboardSpinUpCommand(),
        Commands.waitUntil(shooter::isVelocityWithinTolerance),
        feed());
  }

  /**
   * Runs the full feed sequence:
   *
   * <p>1. Short settle delay — gives the flywheel a moment to recover from any load spike before
   * the ball enters.
   *
   * <p>2. Feeder motor spins up to FEED speed.
   *
   * <p>3. After 0.35 s (once the ball is committed to the feeder) the floor and intake agitate in
   * parallel to push the next game piece forward.
   */
  private Command feed() {
    return Commands.sequence(
        Commands.parallel(
            feeder.feedCommand(),
            Commands.waitSeconds(0.35)
                .andThen(floor.feedCommand())
                .alongWith(intake.agitateCommand())));
  }

  public Command hoodUp() {
    return hood.positionCommand(0.7);
  }

  public Command shootOrShuttle() {
    return Commands.defer(
        () -> {
          double xInches = Units.metersToInches(swerve.getPose().getX());
          double yInches = Units.metersToInches(swerve.getPose().getY());
          boolean isBlueAlliance =
              DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(false);

          final double FUNNEL_X_MIN = 265.0;
          final double FUNNEL_X_MAX = 385.0;
          final double FUNNEL_Y_MIN = 121.30;
          final double FUNNEL_Y_MAX = 195.34;

          boolean inFunnelZone =
              xInches > FUNNEL_X_MIN
                  && xInches < FUNNEL_X_MAX
                  && yInches > FUNNEL_Y_MIN
                  && yInches < FUNNEL_Y_MAX;

          final double ALLIANCE_ZONE_DEPTH_INCHES = 158.32;
          boolean inOwnAllianceZone =
              isBlueAlliance
                  ? xInches < ALLIANCE_ZONE_DEPTH_INCHES
                  : xInches > (650.12 - ALLIANCE_ZONE_DEPTH_INCHES);

          if (inFunnelZone) {
            return Commands.none();
          }
          return inOwnAllianceZone ? aimAndShoot() : shuttleFuel();
        },
        Set.of(swerve, vision, shooter, hood, feeder, floor, intake));
  }

  public Command faceNearestBump() {
    return Commands.defer(
        () -> {
          double currentDeg = swerve.getHeading().getDegrees();
          double[] bumpAngles = {45.0, 135.0, 225.0, 315.0};
          double nearest = bumpAngles[0];
          double minDiff = Double.MAX_VALUE;
          for (double angle : bumpAngles) {
            double diff =
                Math.abs(edu.wpi.first.math.MathUtil.inputModulus(currentDeg - angle, -180, 180));
            if (diff < minDiff) {
              minDiff = diff;
              nearest = angle;
            }
          }
          return vision.rotateToHeadingWhileDriving(
              forwardInput, leftInput, Rotation2d.fromDegrees(nearest));
        },
        Set.of(swerve, vision));
  }

  /** Force-feeds without any vision or shooter readiness check — unjams feeder. */
  public Command manualFeed() {
    return Commands.parallel(feeder.feedCommand(), floor.feedCommand());
  }

  /** Kills shooter and feeder immediately. */
  public Command stopAll() {
    return Commands.parallel(
            shooter.stopCommand(), feeder.stopCommand(), floor.stopCommand(), intake.stopCommand())
        .ignoringDisable(true);
  }

  /** Blasts shooter at full RPM + force feeds to punch a stuck ball through. */
  public Command unjamShooter() {
    return Commands.parallel(
        shooter.punchThroughCommand(), feeder.feedCommand(), floor.feedCommand());
  }

  public Command autoShoot() {
    final PrepareShotCommand prepareShotCommand =
        new PrepareShotCommand(shooter, hood, () -> swerve.getPose());

    return Commands.race(
            Commands.waitSeconds(0.20).andThen(prepareShotCommand),
            Commands.sequence(Commands.waitUntil(prepareShotCommand::isReadyToShoot), feed()))
        .withTimeout(6.0);
  }
}
