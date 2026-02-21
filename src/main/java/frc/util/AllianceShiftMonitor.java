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

  private LEDs.HubState currentHub = LEDs.HubState.HUB_ACTIVE;

  private static final Time WARNING = Seconds.of(5);
  private static final Time SHIFT_QUIET = Seconds.of(20);
  private static final Time RUMBLE_PULSE = Seconds.of(0.3);
  private static final Time RUMBLE_GAP = Seconds.of(0.15);

  private static final double RUMBLE_INTENSITY = 0.7;

  private final CommandXboxController driverController;

  private boolean hasGameData = false;
  private boolean isRedAlliance = false;
  private boolean isOurHubActiveInOddShifts = true;
  private boolean isScheduled = false;
  private Command scheduledCommand = null;
  private final LEDs leds;

  public AllianceShiftMonitor(CommandXboxController driverController, LEDs leds) {
    this.driverController = driverController;
    this.leds = leds;
    // Set initial state to GREEN (for autonomous)
    setHub(LEDs.HubState.HUB_ACTIVE);
  }

  public void periodic() {
    DriverStation.getAlliance().ifPresent(a -> isRedAlliance = a == DriverStation.Alliance.Red);

    // Handle disabled state - only set DISABLED when truly disabled (not auto or teleop)
    if (DriverStation.isDisabled()) {
      if (currentHub != LEDs.HubState.DISABLED) {
        setHub(LEDs.HubState.DISABLED);
      }
      return; // Don't process anything else when disabled
    }

    // Set green during autonomous and reset scheduled flag for new match
    if (DriverStation.isAutonomous()) {
      if (currentHub != LEDs.HubState.HUB_ACTIVE) {
        setHub(LEDs.HubState.HUB_ACTIVE);
      }
      // Reset for new match - allow commands to be scheduled again in teleop
      if (isScheduled) {
        isScheduled = false;
        if (scheduledCommand != null) {
          scheduledCommand.cancel();
          scheduledCommand = null;
        }
      }
    }

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

    updateDashboard();
  }

  private void setHub(LEDs.HubState hub) {
    if (hub == currentHub) return; // Prevent spamming

    currentHub = hub;

    // Update LEDs
    leds.setState(hub);

    // Update SmartDashboard
    SmartDashboard.putString("Shift/Hub Active", hub.name());
  }

  /** Loser: our hub starts first (shifts 1, 3). Opponent gets shifts 2, 4. */
  private Command buildLoserCommand() {
    return Commands.sequence(
        // TRANSITION: 20s blue, then 5s yellow warning
        Commands.runOnce(() -> setHub(LEDs.HubState.TRANSITION)),
        wait(Seconds.of(20)),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_STARTING_SOON)),
        Commands.parallel(
            // rumble3x(),
            wait(WARNING)),

        // SHIFT 1 (ours): 20s green, then 5s purple warning
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ACTIVE)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ENDING_SOON)),
        Commands.parallel(
            // rumble(WARNING),
            wait(WARNING)),

        // SHIFT 2 (opponent): 20s red, then 5s yellow warning
        Commands.runOnce(() -> setHub(LEDs.HubState.OPPONENT_HUB)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_STARTING_SOON)),
        Commands.parallel(
            // rumble3x(),
            wait(WARNING)),

        // SHIFT 3 (ours): 20s green, then 5s purple warning
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ACTIVE)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ENDING_SOON)),
        Commands.parallel(
            // rumble(WARNING),
            wait(WARNING)),

        // SHIFT 4 (opponent): 25s red (no more shifts after this)
        Commands.runOnce(() -> setHub(LEDs.HubState.TRANSITION))
        // Total: 25 + 25 + 25 + 25 + 25 = 125s (+ 20s AUTO = 145s, but match is only 135s)
        );
  }

  /** Winner: opponent's hub starts first (shifts 1, 3). We get shifts 2, 4. */
  private Command buildWinnerCommand() {
    return Commands.sequence(
        // TRANSITION: 25s blue
        Commands.runOnce(() -> setHub(LEDs.HubState.TRANSITION)),
        wait(Seconds.of(25)),

        // SHIFT 1 (opponent): 20s red, then 5s yellow (our hub starting soon)
        Commands.runOnce(() -> setHub(LEDs.HubState.OPPONENT_HUB)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_STARTING_SOON)),
        Commands.parallel(
            // rumble3x(),
            wait(WARNING)),

        // SHIFT 2 (ours): 20s green, then 5s purple (our hub ending soon)
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ACTIVE)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_ENDING_SOON)),
        Commands.parallel(
            // rumble(WARNING),
            wait(WARNING)),

        // SHIFT 3 (opponent): 20s red, then 5s yellow (our hub starting soon)
        Commands.runOnce(() -> setHub(LEDs.HubState.OPPONENT_HUB)),
        wait(SHIFT_QUIET),
        Commands.runOnce(() -> setHub(LEDs.HubState.HUB_STARTING_SOON)),
        Commands.parallel(
            // rumble3x(),
            wait(WARNING)),

        // SHIFT 4 (ours): 20s green, then 5s purple (our hub ending soon)
        Commands.runOnce(() -> setHub(LEDs.HubState.TRANSITION)),
        wait(SHIFT_QUIET),
        Commands.parallel(
            // rumble(WARNING),
            wait(WARNING))
        // Total: 25 + 25 + 25 + 25 + 25 = 125s (+ 20s AUTO = 145s, but match is only 135s)
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
    setHub(LEDs.HubState.HUB_ACTIVE); // Back to GREEN for autonomous
  }
}
