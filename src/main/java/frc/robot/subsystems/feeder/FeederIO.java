package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean kickerConnected = false;
    public double kickerVelocityRotPerSec = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run kicker motor at velocity (rot/s). */
  public default void setKickerVelocity(double velocityRotPerSec) {}

  /** Stop kicker motor. */
  public default void stop() {}
}
