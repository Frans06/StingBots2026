package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private boolean running = false;

  private final Alert kickerDisconnected =
      new Alert("Feeder kicker motor disconnected.", AlertType.kError);

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    if (DriverStation.isDisabled()) {
      io.stop();
      running = false;
    }

    kickerDisconnected.set(!inputs.kickerConnected);
  }

  /** Start the kicker at the default feed velocity. */
  public void kick() {
    running = true;
    io.setKickerVelocity(FeederConstants.KICKER_VELOCITY);
  }

  public void stop() {
    running = false;
    io.stop();
  }

  @AutoLogOutput(key = "Feeder/IsRunning")
  public boolean isRunning() {
    return running;
  }

  public double getKickerVelocity() {
    return inputs.kickerVelocityRotPerSec;
  }
}
