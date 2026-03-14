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

  private boolean hasGameData = false;
  private boolean isRedAlliance = false;
  private boolean isOurHubActiveInOddShifts = true;
  private boolean isScheduled = false;
  private Command scheduledCommand = null;

  public AllianceShiftMonitor(CommandXboxController driverController) {
    this.driverController = driverController;
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

    updateDashboard();
  }

  /** Loser: odd shifts (1, 3) are ours. */
  private Command buildLoserCommand() {
    return Commands.sequence(
        // TRANSITION: 10s total, rumble3x at 5s mark
        Commands.deadline(wait(Seconds.of(10)), Commands.sequence(wait(WARNING), rumble3x())),
        // SHIFT 1 (ours): 20s quiet + 5s rumble = 25s total
        wait((SHIFT_QUIET)),
        rumble((WARNING)),
        // SHIFT 2 (opponent): 25s total, rumble3x at 20s mark
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 3 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING),
        // SHIFT 4 (opponent): 25s total, rumble3x at 20s mark
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x()))
        // Total: 10 + 25 + 25 + 25 + 25 = 110s (+ 20s AUTO = 130s)
        );
  }

  /** Winner: even shifts (2, 4) are ours. */
  private Command buildWinnerCommand() {
    return Commands.sequence(
        // TRANSITION: 10s total, rumble at 5s mark (transition ending soon)
        Commands.deadline(wait(Seconds.of(10)), Commands.sequence(wait(WARNING), rumble(WARNING))),
        // SHIFT 1 (opponent): 25s total, rumble3x at 20s mark (for our shift 2)
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 2 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING),
        // SHIFT 3 (opponent): 25s total, rumble3x at 20s mark (for our shift 4)
        Commands.deadline(wait(Seconds.of(25)), Commands.sequence(wait(SHIFT_QUIET), rumble3x())),
        // SHIFT 4 (ours): 20s quiet + 5s rumble = 25s total
        wait(SHIFT_QUIET),
        rumble(WARNING)
        // Total: 10 + 25 + 25 + 25 + 25 = 110s (+ 20s AUTO = 130s)
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

  // ── Hub active logic ──────────────────────────────────────────────────────

  /**
   * Returns true only when the current shift is genuinely ours — no buffer applied. Use this for
   * LED indicators and dashboard display.
   */
  public boolean isOurHubActiveRaw() {
    if (!hasGameData) return true;
    Phase phase = Phase.fromMatchTime(DriverStation.getMatchTime());
    return switch (phase) {
      case TRANSITION -> true;
      case END_GAME -> true;
      case SHIFT_1, SHIFT_3 -> isOurHubActiveInOddShifts;
      case SHIFT_2, SHIFT_4 -> !isOurHubActiveInOddShifts;
    };
  }

  private static final double SHOOT_WINDOW_BUFFER = 2.0; // seconds grace period each side

  /**
   * Returns true when it is safe to shoot — our shift, plus a 2s grace window on each boundary to
   * avoid penalising shots that were already in flight. Use this to gate the shoot command.
   */
  public boolean isOurHubActive() {
    if (!hasGameData) return true;
    double matchTime = DriverStation.getMatchTime();
    Phase phase = Phase.fromMatchTime(matchTime);
    double timeRemainingInShift = matchTime - phase.seconds;

    if (isOurHubActiveRaw()) return true;

    // 2s carry-over at the start of an inactive shift (shot was already spinning up)
    boolean inLeadingBuffer = timeRemainingInShift >= (25.0 - SHOOT_WINDOW_BUFFER);
    // 2s pre-window before the inactive shift ends (next shift is ours)
    boolean inTrailingBuffer = timeRemainingInShift <= SHOOT_WINDOW_BUFFER;

    return inLeadingBuffer || inTrailingBuffer;
  }

  // ── Next shift info ───────────────────────────────────────────────────────

  /**
   * Returns the name of the next shift that belongs to our alliance, prefixed with the alliance
   * colour. e.g. "RED: Shift 3" or "BLUE: Shift 2". If we are currently in an active shift, returns
   * the NEXT future active shift. Returns "–" when no future active shift exists (end of match or
   * no game data).
   */
  public String getNextOurShiftLabel() {
    if (!hasGameData) return "–";
    double matchTime = DriverStation.getMatchTime();
    Phase current = Phase.fromMatchTime(matchTime);
    String allianceColor = isRedAlliance ? "RED" : "BLUE";

    Phase[] ordered = {
      Phase.TRANSITION, Phase.SHIFT_1, Phase.SHIFT_2, Phase.SHIFT_3, Phase.SHIFT_4, Phase.END_GAME
    };

    boolean passedCurrent = false;
    for (Phase p : ordered) {
      if (!passedCurrent) {
        if (p == current) passedCurrent = true;
        continue; // skip up to and including current phase
      }
      boolean ours =
          switch (p) {
            case TRANSITION -> true;
            case END_GAME -> false;
            case SHIFT_1, SHIFT_3 -> isOurHubActiveInOddShifts;
            case SHIFT_2, SHIFT_4 -> !isOurHubActiveInOddShifts;
          };
      if (ours) {
        String label =
            switch (p) {
              case TRANSITION -> "Transition";
              case SHIFT_1 -> "Shift 1";
              case SHIFT_2 -> "Shift 2";
              case SHIFT_3 -> "Shift 3";
              case SHIFT_4 -> "Shift 4";
              case END_GAME -> "End Game";
            };
        return allianceColor + ": " + label;
      }
    }
    return "–";
  }

  // ── Timing helpers ────────────────────────────────────────────────────────

  /** Seconds remaining in the current shift, clamped to 0–25. */
  public double getSecondsRemainingInShift() {
    if (!DriverStation.isFMSAttached()) return 25.0;
    double matchTime = DriverStation.getMatchTime();
    Phase phase = Phase.fromMatchTime(matchTime);
    return Math.max(0.0, matchTime - phase.seconds);
  }

  /**
   * Shift countdown as a 0.0–1.0 fraction (1.0 = shift just started, 0.0 = shift over). Suitable
   * for a Shuffleboard progress bar widget set to range 0–1.
   */
  public double getShiftProgress() {
    if (!DriverStation.isTeleopEnabled()) return 0.0;
    return Math.max(0.0, Math.min(1.0, getSecondsRemainingInShift() / 25.0));
  }

  // ── Dashboard ─────────────────────────────────────────────────────────────

  private void updateDashboard() {
    String phase = getPhase();

    SmartDashboard.putString("Shift/Phase", phase);
    SmartDashboard.putString("Shift/Alliance", isRedAlliance ? "RED" : "BLUE");
    SmartDashboard.putBoolean("Shift/GameDataReceived", hasGameData);
    SmartDashboard.putBoolean("Shift/Scheduled", isScheduled);

    // Raw: is the hub genuinely ours right now (no grace buffer)?
    SmartDashboard.putBoolean("Shift/OurHubActiveRaw", isOurHubActiveRaw());
    // Buffered: safe to shoot (includes 2s grace windows at shift boundaries)?
    SmartDashboard.putBoolean("Shift/OurHubActive", isOurHubActive());

    // Countdown within the current shift
    SmartDashboard.putNumber("Shift/Time Left", getSecondsRemainingInShift());
    SmartDashboard.putNumber("Shift/ShiftProgress", getShiftProgress());

    // Next shift that is ours, with alliance colour prefix
    SmartDashboard.putString("Shift/NextOurShift", getNextOurShiftLabel());
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
