package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  public enum RoutingMode {
    BOTH_SHOOTERS,
    LEFT_ONLY,
    RIGHT_ONLY,
    OFF
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private RoutingMode currentMode = RoutingMode.OFF;

  private final Alert feedDisconnected =
      new Alert("Indexer feed motor disconnected.", AlertType.kError);
  private final Alert leftShaftDisconnected =
      new Alert("Indexer left shaft motor disconnected.", AlertType.kError);
  private final Alert rightShaftDisconnected =
      new Alert("Indexer right shaft motor disconnected.", AlertType.kError);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (DriverStation.isDisabled()) {
      io.stop();
      currentMode = RoutingMode.OFF;
    }

    feedDisconnected.set(!inputs.feedConnected);
    leftShaftDisconnected.set(!inputs.leftShaftConnected);
    rightShaftDisconnected.set(!inputs.rightShaftConnected);
  }

  /** Set the routing mode. Controls which mecanum shafts spin and in which direction. */
  public void setRoutingMode(RoutingMode mode) {
    currentMode = mode;
    switch (mode) {
      case BOTH_SHOOTERS:
        io.setFeedVelocity(IndexerConstants.FEED_VELOCITY);
        io.setLeftShaftVelocity(IndexerConstants.SHAFT_VELOCITY);
        io.setRightShaftVelocity(IndexerConstants.SHAFT_VELOCITY);
        break;
      case LEFT_ONLY:
        io.setFeedVelocity(IndexerConstants.FEED_VELOCITY);
        io.setLeftShaftVelocity(IndexerConstants.SHAFT_VELOCITY);
        io.setRightShaftVelocity(0.0);
        break;
      case RIGHT_ONLY:
        io.setFeedVelocity(IndexerConstants.FEED_VELOCITY);
        io.setLeftShaftVelocity(0.0);
        io.setRightShaftVelocity(IndexerConstants.SHAFT_VELOCITY);
        break;
      case OFF:
        io.stop();
        break;
    }
  }

  public void stop() {
    currentMode = RoutingMode.OFF;
    io.stop();
  }

  @AutoLogOutput(key = "Indexer/RoutingMode")
  public String getRoutingModeString() {
    return currentMode.name();
  }

  public RoutingMode getRoutingMode() {
    return currentMode;
  }
}
