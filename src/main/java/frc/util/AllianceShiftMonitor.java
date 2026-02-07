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
    TRANSITION(130),
    SHIFT_1(105),
    SHIFT_2(80),
    SHIFT_3(55),
    SHIFT_4(30),
    END_GAME(0);

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
  private static final Time TRIPLE_RUMBLE = RUMBLE_PULSE.times(3).plus(RUMBLE_GAP.times(2));
  private static final Time POST_TRIPLE_GAP = WARNING.minus(TRIPLE_RUMBLE);

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

  /**
   * Loser: odd shifts (1, 3) are ours.
   *
   * <pre>
   * rumble3x      -> transition: our shift 1 coming
   * wait 3.8s     -> finish transition
   * wait 20s      -> shift 1 (ours): quiet
   * rumble 5s     -> shift 1 ending
   * wait 20s      -> shift 2 (opponent): quiet
   * rumble3x      -> our shift 3 coming
   * wait 3.8s     -> finish shift 2
   * wait 20s      -> shift 3 (ours): quiet
   * rumble 5s     -> shift 3 ending
   * wait 20s      -> shift 4 (opponent): quiet
   * rumble3x      -> end game coming
   * </pre>
   */
  private Command buildLoserCommand() {
    return Commands.sequence(
        rumble3x(),
        wait(POST_TRIPLE_GAP),
        wait(SHIFT_QUIET),
        rumble(WARNING),
        wait(SHIFT_QUIET),
        rumble3x(),
        wait(POST_TRIPLE_GAP),
        wait(SHIFT_QUIET),
        rumble(WARNING),
        wait(SHIFT_QUIET),
        rumble3x());
  }

  /**
   * Winner: even shifts (2, 4) are ours.
   *
   * <pre>
   * rumble 5s     -> transition: scoring window ending
   * wait 20s      -> shift 1 (opponent): quiet
   * rumble3x      -> our shift 2 coming
   * wait 3.8s     -> finish shift 1
   * wait 20s      -> shift 2 (ours): quiet
   * rumble 5s     -> shift 2 ending
   * wait 20s      -> shift 3 (opponent): quiet
   * rumble3x      -> our shift 4 coming
   * wait 3.8s     -> finish shift 3
   * wait 20s      -> shift 4 (ours): quiet
   * rumble 5s     -> shift 4 ending
   * </pre>
   */
  private Command buildWinnerCommand() {
    return Commands.sequence(
        rumble(WARNING),
        wait(SHIFT_QUIET),
        rumble3x(),
        wait(POST_TRIPLE_GAP),
        wait(SHIFT_QUIET),
        rumble(WARNING),
        wait(SHIFT_QUIET),
        rumble3x(),
        wait(POST_TRIPLE_GAP),
        wait(SHIFT_QUIET),
        rumble(WARNING));
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
  }
}
