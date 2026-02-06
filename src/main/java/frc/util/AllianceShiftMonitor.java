package frc.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.HubState;

public class AllianceShiftMonitor {

  private static final double TRANSITION_END = 130.0; // 2:10
  private static final double SHIFT_1_END = 105.0;
  private static final double SHIFT_2_END = 80.0;
  private static final double SHIFT_3_END = 55.0;
  private static final double SHIFT_4_END = 30.0;

  private static final double WARNING_TIME = 5.0;

  private static final double RUMBLE_INTENSITY = 0.7;
  private static final double RUMBLE_DURATION = 0.3;
  private static final double RUMBLE_GAP = 0.15;

  private final CommandXboxController driverController;
  private final LEDs leds;

  private boolean gameDataReceived = false;
  private boolean isRedAlliance = false;
  private boolean ourHubActiveInOddShifts = true;

  private int currentShift = -1;

  private boolean transitionHandled = false;
  private boolean continuousRumbleActive = false;

  // Flags to avoid repeating rumbles
  private boolean startRumbleTriggered = false;

  @SuppressWarnings("unused")
  private boolean endRumbleTriggered = false;

  public AllianceShiftMonitor(CommandXboxController driverController, LEDs leds) {
    this.driverController = driverController;
    this.leds = leds;
  }

  public void periodic() {
    updateAlliance();
    updateGameData();

    if (!DriverStation.isTeleopEnabled()) {
      stopContinuousRumble();
      leds.setColor(HubState.DISABLED);
      ;
      updateDashboard();
      return;
    }

    double time = DriverStation.getMatchTime();
    int newShift = getShift(time);

    if (newShift != currentShift) {
      currentShift = newShift;
      transitionHandled = false;
      stopContinuousRumble();
      startRumbleTriggered = false;
      endRumbleTriggered = false;
    }

    // TRANSITION ONLY
    if (currentShift == 0) {
      handleTransitionRumble(time);
      updateDashboard();
      return;
    }

    // Our Alliance Shift rumble
    handleAllianceShiftRumble(time);

    updateDashboard();
  }

  // Our Alliance Shift rumble
  private void handleAllianceShiftRumble(double time) {
    if (!gameDataReceived) return;

    double shiftEnd = getNextShiftTime(currentShift);

    // Default state
    if (isHubActiveNow()) {
      leds.setColor(HubState.HUB_ACTIVE);
    } else {
      leds.setColor(HubState.OPPONENT_HUB);
    }

    // CASE 1: We're currently in OPPONENT'S shift, but OUR shift is coming up next
    // Triple rumble 5s before OUR shift starts
    if (!isHubActiveNow() && isHubActiveInShift(currentShift + 1)) {
      if (!startRumbleTriggered
          && shiftEnd > 0
          && time <= shiftEnd + WARNING_TIME
          && time > shiftEnd) {
        rumbleTriple();
        startRumbleTriggered = true;
      }
    }

    // CASE 2: We're currently in OUR shift
    // Continuous rumble 5s before OUR shift ends
    if (isHubActiveNow()) {
      if (!continuousRumbleActive
          && shiftEnd > 0
          && time <= shiftEnd + WARNING_TIME
          && time > shiftEnd) {
        startContinuousRumble();
        endRumbleTriggered = true;
      }

      // Stop continuous rumble once shift ends (time drops below shiftEnd)
      if (continuousRumbleActive && time <= shiftEnd) {
        stopContinuousRumble();
      }
    }
  }

  // Transition Logic
  private void handleTransitionRumble(double time) {
    if (transitionHandled || !gameDataReceived) return;

    double end = TRANSITION_END;

    if (time <= end + WARNING_TIME && time > end) {
      boolean ourShiftNext = isHubActiveInShift(1);

      if (ourShiftNext) {
        rumbleTriple();
        leds.setColor(HubState.HUB_STARTING_SOON);
      } else {
        startContinuousRumble();
        leds.setColor(HubState.OPPONENT_HUB);
      }

      transitionHandled = true;
    }

    if (time <= end) {
      stopContinuousRumble();
    }
  }

  // Hub/Shift Logic
  private boolean isHubActiveInShift(int shift) {
    if (shift == 0 || shift == 5) return true;

    boolean isOdd = (shift == 1 || shift == 3);
    return (isOdd && ourHubActiveInOddShifts) || (!isOdd && !ourHubActiveInOddShifts);
  }

  private boolean isHubActiveNow() {
    return isHubActiveInShift(currentShift);
  }

  private void updateAlliance() {
    var alliance = DriverStation.getAlliance();
    alliance.ifPresent(a -> isRedAlliance = a == DriverStation.Alliance.Red);
  }

  private void updateGameData() {
    if (gameDataReceived) return;

    String data = DriverStation.getGameSpecificMessage();
    if (data != null && !data.isEmpty()) {
      char winner = data.charAt(0);
      ourHubActiveInOddShifts = isRedAlliance ? (winner == 'B') : (winner == 'R');
      gameDataReceived = true;
    }
  }

  private int getShift(double time) {
    if (time >= TRANSITION_END) return 0;
    if (time >= SHIFT_1_END) return 1;
    if (time >= SHIFT_2_END) return 2;
    if (time >= SHIFT_3_END) return 3;
    if (time >= SHIFT_4_END) return 4;
    return 5;
  }

  // Rumble
  private void startContinuousRumble() {
    if (continuousRumbleActive) return;
    driverController.getHID().setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
    continuousRumbleActive = true;
  }

  private void stopContinuousRumble() {
    if (!continuousRumbleActive) return;
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    continuousRumbleActive = false;
  }

  private Command rumbleCommand(double intensity) {
    return new RunCommand(
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, intensity))
        .finallyDo(i -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  private void rumbleTriple() {
    Commands.sequence(
            rumbleCommand(RUMBLE_INTENSITY).withTimeout(RUMBLE_DURATION),
            Commands.waitSeconds(RUMBLE_GAP),
            rumbleCommand(RUMBLE_INTENSITY).withTimeout(RUMBLE_DURATION),
            Commands.waitSeconds(RUMBLE_GAP),
            rumbleCommand(RUMBLE_INTENSITY).withTimeout(RUMBLE_DURATION))
        .schedule();
  }

  // Dashboard Values
  private void updateDashboard() {
    SmartDashboard.putString("Shift/Phase", getPhase());
    SmartDashboard.putString("Shift/Status", getStatusString());
    SmartDashboard.putString("Shift/Alliance", isRedAlliance ? "RED" : "BLUE");

    SmartDashboard.putBoolean("Shift/Hub Active Now", isHubActiveNow());
    SmartDashboard.putBoolean("Shift/Hub Active Next", isHubActiveInShift(currentShift + 1));
    SmartDashboard.putBoolean("Shift/Game Data Received", gameDataReceived);

    double time = DriverStation.getMatchTime();
    double end = getNextShiftTime(currentShift);
    if (end > 0) {
      SmartDashboard.putNumber("Shift/Time Left", Math.max(0, time - end));
    }
  }

  private String getStatusString() {
    if (!gameDataReceived) return "WAITING FOR GAME DATA";
    if (currentShift == 0) return "TRANSITION";
    if (isHubActiveNow()) return "OUR HUB ACTIVE";
    return "OPPONENT HUB ACTIVE";
  }

  public String getPhase() {
    double t = DriverStation.getMatchTime();
    if (!DriverStation.isTeleopEnabled()) return "DISABLED";
    if (t >= TRANSITION_END) return "TRANSITION";
    if (t >= SHIFT_1_END) return "SHIFT 1";
    if (t >= SHIFT_2_END) return "SHIFT 2";
    if (t >= SHIFT_3_END) return "SHIFT 3";
    if (t >= SHIFT_4_END) return "SHIFT 4";
    return "END GAME";
  }

  private double getNextShiftTime(int shift) {
    switch (shift) {
      case 0:
        return TRANSITION_END;
      case 1:
        return SHIFT_1_END;
      case 2:
        return SHIFT_2_END;
      case 3:
        return SHIFT_3_END;
      case 4:
        return SHIFT_4_END;
      default:
        return -1;
    }
  }

  public void reset() {
    gameDataReceived = false;
    transitionHandled = false;
    currentShift = -1;
    stopContinuousRumble();
    startRumbleTriggered = false;
    endRumbleTriggered = false;
  }
}
