package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AllianceShiftMonitor extends SubsystemBase {

  private enum Phase {
    TRANSITION(130), // 2:10 remaining (after 20s AUTO)
    SHIFT_1(105), // 1:45 remaining
    SHIFT_2(80), // 1:20 remaining
    SHIFT_3(55), // 0:55 remaining
    SHIFT_4(30), // 0:30 remaining
    END_GAME(0); // 0:00 remaining

    private static final double WARNING_TIME = 5.0;
    private final double seconds;

    private Phase(double seconds) {
      this.seconds = seconds;
    }

    private double warningThreshold() {
      return seconds - WARNING_TIME;
    }

    private static Phase fromMatchTime(double seconds) {
      for (Phase p : values()) {
        if (seconds >= p.seconds) return p;
      }
      return END_GAME;
    }
  }

  // Loser pattern (odd shifts ours):  triple, sustained, triple, sustained, triple
  // Winner pattern (even shifts ours): sustained, triple, sustained, triple, sustained
  private static final Time RUMBLE_DURATION = Seconds.of(5);
  private static final Time RUMBLE_PULSE = Seconds.of(0.3);
  private static final Time RUMBLE_GAP = Seconds.of(0.15);
  private static final double RUMBLE_INTENSITY = 0.7;

  private final CommandXboxController driverController;
  private final boolean[] firedPhases = new boolean[Phase.values().length - 1]; // exclude END_GAME

  private boolean hasGameData = false;
  private boolean isOddShiftsOurs = true;

  public AllianceShiftMonitor(CommandXboxController driverController) {
    this.driverController = driverController;
  }

  @Override
  public void periodic() {
    getGameData();
    handleShiftWarnings();
    updateDashboard();
  }

  private Command rumble(Time duration) {
    Command cmd =
        new RunCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY))
            .finallyDo(i -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
            .withTimeout(duration.in(Seconds));
    cmd.addRequirements(this);
    return cmd;
  }

  private Command rumble3x() {
    Command cmd =
        Commands.sequence(
            rumble(RUMBLE_PULSE),
            Commands.waitSeconds(RUMBLE_GAP.in(Seconds)),
            rumble(RUMBLE_PULSE),
            Commands.waitSeconds(RUMBLE_GAP.in(Seconds)),
            rumble(RUMBLE_PULSE));
    cmd.addRequirements(this);
    return cmd;
  }

  private void getGameData() {
    if (hasGameData) return;
    String data = DriverStation.getGameSpecificMessage();
    if (data == null || data.isEmpty()) return;

    boolean isRed =
        DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
    char winner = data.charAt(0);
    isOddShiftsOurs = isRed ? (winner == 'B') : (winner == 'R');
    hasGameData = true;
  }

  private void handleShiftWarnings() {
    if (!DriverStation.isTeleopEnabled() || !hasGameData) return;

    double seconds = DriverStation.getMatchTime();
    Phase[] phases = Phase.values();
    for (int i = 0; i < firedPhases.length; i++) {
      if (firedPhases[i] || seconds > phases[i].warningThreshold()) continue;

      firedPhases[i] = true;
      boolean isOurShiftNext = (i % 2 == 0) == isOddShiftsOurs;
      if (isOurShiftNext) {
        rumble3x().schedule();
      } else {
        rumble(RUMBLE_DURATION).schedule();
      }
    }
  }

  private void updateDashboard() {
    String phase = getPhase();

    SmartDashboard.putString("Shift/Phase", phase);
    SmartDashboard.putString(
        "Shift/Alliance",
        DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red ? "RED" : "BLUE")
            .orElse("UNKNOWN"));
    SmartDashboard.putBoolean("Shift/Game Data Received", hasGameData);

    if (!"DISABLED".equals(phase)) {
      double seconds = DriverStation.getMatchTime();
      Phase p = Phase.fromMatchTime(seconds);
      SmartDashboard.putNumber("Shift/Time Left", Math.max(0, seconds - p.seconds));
    }
  }

  private String getPhase() {
    if (!DriverStation.isTeleopEnabled()) return "DISABLED";
    return Phase.fromMatchTime(DriverStation.getMatchTime()).name();
  }

  public void reset() {
    hasGameData = false;
    for (int i = 0; i < firedPhases.length; i++) {
      firedPhases[i] = false;
    }
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }
}
