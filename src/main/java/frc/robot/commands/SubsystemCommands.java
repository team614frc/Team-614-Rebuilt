package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVisualizer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.Set;
import java.util.function.DoubleSupplier;

public final class SubsystemCommands {
  private final Swerve swerve;
  private final Intake intake;
  private final Floor floor;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Hood hood;
  private final Hanger hanger;
  private final Vision vision;
  private final ShooterVisualizer shooterVisualizer;

  private final DoubleSupplier forwardInput;
  private final DoubleSupplier leftInput;

  public SubsystemCommands(
      Swerve swerve,
      Intake intake,
      Floor floor,
      Feeder feeder,
      Shooter shooter,
      Hood hood,
      Hanger hanger,
      Vision vision,
      ShooterVisualizer shooterVisualizer,
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
    this.shooterVisualizer = shooterVisualizer;

    this.forwardInput = forwardInput;
    this.leftInput = leftInput;
  }

  public SubsystemCommands(
      Swerve swerve,
      Intake intake,
      Floor floor,
      Feeder feeder,
      Shooter shooter,
      Hood hood,
      Hanger hanger,
      Vision vision) {
    this(swerve, intake, floor, feeder, shooter, hood, hanger, vision, null, () -> 0, () -> 0);
  }

  public Command aimAndShoot() {
    return Commands.defer(
        () -> {
          final PrepareShot prepareShot = new PrepareShot(shooter, hood, () -> swerve.getPose());

          return Commands.parallel(
              vision.rotateToAllianceTagWhileDriving(forwardInput, leftInput),
              Commands.waitSeconds(0.25).andThen(prepareShot),
              Commands.sequence(
                  Commands.waitUntil(
                      () -> {
                        boolean aimed = vision.isAimed();
                        boolean ready = prepareShot.isReadyToShoot();
                        return aimed && ready;
                      }),
                  // SIMULATION: Continuously fire while held
                  // REAL ROBOT: Fire once with feed() command
                  RobotBase.isSimulation()
                      ? Commands.repeatingSequence(
                          Commands.runOnce(
                              () -> {
                                if (shooterVisualizer != null) {
                                  shooterVisualizer.launchFuel();
                                  System.out.println(
                                      "=== FIRED 3 BALLS, remaining: "
                                          + shooterVisualizer.getFuelCount()
                                          + " ===");
                                }
                              }),
                          Commands.waitSeconds(0.3) // Fire 3 balls every 0.3 seconds
                          )
                      : Commands.sequence(
                          Commands.runOnce(
                              () -> System.out.println("=== WAIT COMPLETE - FEEDING ===")),
                          feed())));
        },
        Set.of(vision, shooter, hood));
  }

  public Command shootManually() {
    return shooter.dashboardSpinUpCommand().andThen(feed()).handleInterrupt(() -> shooter.stop());
  }

  private Command feed() {
    return Commands.sequence(
        Commands.waitSeconds(0.25),
        Commands.parallel(
            feeder.feedCommand(),
            Commands.waitSeconds(0.125)
                .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))));
  }
}
