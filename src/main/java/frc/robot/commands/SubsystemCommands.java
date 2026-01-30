package frc.robot.commands;

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
      VisionSubsystem vision) {
    this(swerve, intake, floor, feeder, shooter, hood, hanger, vision, () -> 0, () -> 0);
  }

  public Command aimAndShoot() {
    // Use defer() instead of deferredProxy()
    return Commands.defer(
        () -> {
          final PrepareShotCommand prepareShotCommand =
              new PrepareShotCommand(shooter, hood, () -> swerve.getPose());

          return Commands.parallel(
              vision.rotateToAllianceTagWhileDriving(forwardInput, leftInput),
              Commands.waitSeconds(0.25).andThen(prepareShotCommand),
              Commands.waitUntil(() -> vision.isAimed() && prepareShotCommand.isReadyToShoot())
                  .andThen(feed()));
        },
        Set.of(vision, shooter, hood) // Declare required subsystems
        );
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
