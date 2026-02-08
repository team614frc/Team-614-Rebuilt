package frc.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.HubState;

public class AllianceShiftMonitor {

  private enum Phase {
    TRANSITION(130), // 2:10 remaining (after 20s AUTO)
    SHIFT_1(105), // 1:45 remaining
    SHIFT_2(80), // 1:20 remaining
    SHIFT_3(55), // 0:55 remaining
    SHIFT_4(30), // 0:30 remaining
    END_GAME(0); // 0:00 remaining

    private final double seconds;

    private Phase(double seconds) {
      this.seconds = seconds;
    }

    private static Phase fromMatchTime(double seconds) {
      for (Phase phase : values()) {
        if (seconds >= phase.seconds) return phase;
      }
      return END_GAME;
    }
  }

  private static final Time WARNING = Seconds.of(5);
  private static final Time SHIFT_QUIET = Seconds.of(20);
  private static final Time RUMBLE_PULSE = Seconds.of(0.3);
  private static final Time RUMBLE_GAP = Seconds.of(0.15);

  private static final double RUMBLE_INTENSITY = 0.7;

  private final CommandXboxController driverController;
  private final LEDs leds;

  private boolean hasGameData = false;
  private boolean isRedAlliance = false;
  private boolean isOurHubActiveInOddShifts = true;
  private boolean isScheduled = false;
  private Command scheduledCommand = null;

  public AllianceShiftMonitor(CommandXboxController driverController, LEDs leds) {
    this.driverController = driverController;
    this.leds = leds;
  }

  public void periodic() {
    DriverStation.getAlliance().ifPresent(a -> isRedAlliance = a == DriverStation.Alliance.Red);

    if (!hasGameData) {
      String data = DriverStation.getGameSpecificMessage();
      if (data != null && !data.isEmpty()) {
        char winner = data.charAt(0);
        isOurHubActiveInOddShifts = isRedAlliance ? (winner == 'B') : (winner == 'R');
        hasGameData = true;
      }
    }

    if (DriverStation.isTeleopEnabled() && hasGameData && !isScheduled) {
      scheduledCommand = isOurHubActiveInOddShifts ? buildLoserCommand() : buildWinnerCommand();
      CommandScheduler.getInstance().schedule(scheduledCommand);
      isScheduled = true;
    }

    if (!DriverStation.isTeleopEnabled()) {
      leds.setState(HubState.DISABLED);
    }

    updateDashboard();
  }

  /** 
   * Loser: odd shifts (1, 3) are ours.
   */
  private Command buildLoserCommand() {
    return Commands.sequence(
        // TRANSITION: opponent hub for 5s, rumble3x + "our shift starting soon" in parallel
        Commands.deadline(
            wait(Seconds.of(10)), 
            Commands.sequence(
                opponentActiveLED().withTimeout(WARNING.in(Seconds)),
                Commands.parallel(rumble3x(), ourShiftNextLED())
            )
        ),
        // SHIFT 1 (ours): hub active for 20s, then rumble 5s (keep hub active during rumble)
        Commands.parallel(
            hubActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
            Commands.sequence(wait(SHIFT_QUIET), Commands.none())
        ),
        Commands.parallel(rumble(WARNING), hubActiveLED().withTimeout(WARNING.in(Seconds))),
        // SHIFT 2 (opponent): opponent for 20s, rumble3x + "our shift starting soon" in parallel
        Commands.deadline(
            wait(Seconds.of(25)), 
            Commands.sequence(
                opponentActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
                Commands.parallel(rumble3x(), ourShiftNextLED())
            )
        ),
        // SHIFT 3 (ours): hub active for 20s, then rumble 5s (keep hub active during rumble)
        Commands.parallel(
            hubActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
            Commands.sequence(wait(SHIFT_QUIET), Commands.none())
        ),
        Commands.parallel(rumble(WARNING), hubActiveLED().withTimeout(WARNING.in(Seconds))),
        // SHIFT 4 (opponent): opponent for 20s, rumble3x + "end game soon" in parallel
        Commands.deadline(
            wait(Seconds.of(25)), 
            Commands.sequence(
                opponentActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
                Commands.parallel(rumble3x(), ourShiftNextLED())
            )
        )
    );
  }

  /** 
   * Winner: even shifts (2, 4) are ours.
   */
  private Command buildWinnerCommand() {
    return Commands.sequence(
        // TRANSITION: hub active for 5s, then rumble 5s (keep hub active during rumble)
        Commands.deadline(
            wait(Seconds.of(10)), 
            Commands.sequence(
                hubActiveLED().withTimeout(WARNING.in(Seconds)),
                Commands.parallel(rumble(WARNING), hubActiveLED())
            )
        ),
        // SHIFT 1 (opponent): opponent for 20s, rumble3x + "our shift starting soon" in parallel
        Commands.deadline(
            wait(Seconds.of(25)), 
            Commands.sequence(
                opponentActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
                Commands.parallel(rumble3x(), ourShiftNextLED())
            )
        ),
        // SHIFT 2 (ours): hub active for 20s, then rumble 5s (keep hub active during rumble)
        Commands.parallel(
            hubActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
            Commands.sequence(wait(SHIFT_QUIET), Commands.none())
        ),
        Commands.parallel(rumble(WARNING), hubActiveLED().withTimeout(WARNING.in(Seconds))),
        // SHIFT 3 (opponent): opponent for 20s, rumble3x + "our shift starting soon" in parallel
        Commands.deadline(
            wait(Seconds.of(25)), 
            Commands.sequence(
                opponentActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
                Commands.parallel(rumble3x(), ourShiftNextLED())
            )
        ),
        // SHIFT 4 (ours): hub active for 20s, then rumble 5s (keep hub active during rumble)
        Commands.parallel(
            hubActiveLED().withTimeout(SHIFT_QUIET.in(Seconds)),
            Commands.sequence(wait(SHIFT_QUIET), Commands.none())
        ),
        Commands.parallel(rumble(WARNING), hubActiveLED().withTimeout(WARNING.in(Seconds)))
    );
  }

  private Command wait(Time duration) {
    return Commands.waitSeconds(duration.in(Seconds));
  }

  private Command rumble(Time duration) {
    return new RunCommand(
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY))
        .finallyDo(i -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(duration.in(Seconds));
  }

  private Command rumble3x() {
    return Commands.sequence(
        rumble(RUMBLE_PULSE),
        wait(RUMBLE_GAP),
        rumble(RUMBLE_PULSE),
        wait(RUMBLE_GAP),
        rumble(RUMBLE_PULSE));
  }

  private Command ourShiftNextLED() {
    return new RunCommand(() -> leds.setState(HubState.HUB_STARTING_SOON));
  }

  private Command hubActiveLED() {
    return new RunCommand(() -> leds.setState(HubState.HUB_ACTIVE));
  }

  private Command opponentActiveLED() {
    return new RunCommand(() -> leds.setState(HubState.OPPONENT_HUB));
  }

  private void updateDashboard() {
    String phase = getPhase();

    SmartDashboard.putString("Shift/Phase", phase);
    SmartDashboard.putString("Shift/Alliance", isRedAlliance ? "RED" : "BLUE");
    SmartDashboard.putBoolean("Shift/Game Data Received", hasGameData);
    SmartDashboard.putBoolean("Shift/Scheduled", isScheduled);

    if (!"DISABLED".equals(phase)) {
      double seconds = DriverStation.getMatchTime();
      Phase p = Phase.fromMatchTime(seconds);
      SmartDashboard.putNumber("Shift/Time Left", Math.max(0, seconds - p.seconds));
    }
  }

  public String getPhase() {
    if (!DriverStation.isTeleopEnabled()) return "DISABLED";
    return Phase.fromMatchTime(DriverStation.getMatchTime()).name();
  }

  public void reset() {
    hasGameData = false;
    isScheduled = false;
    if (scheduledCommand != null) {
      scheduledCommand.cancel();
      scheduledCommand = null;
    }
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }
}