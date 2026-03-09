package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean feedConnected = false;
    public double feedVelocityRotPerSec = 0.0;
    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;

    public boolean leftShaftConnected = false;
    public double leftShaftVelocityRotPerSec = 0.0;
    public double leftShaftAppliedVolts = 0.0;
    public double leftShaftCurrentAmps = 0.0;

    public boolean rightShaftConnected = false;
    public double rightShaftVelocityRotPerSec = 0.0;
    public double rightShaftAppliedVolts = 0.0;
    public double rightShaftCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run feed motor at velocity (rot/s). */
  public default void setFeedVelocity(double velocityRotPerSec) {}

  /** Run left mecanum shaft at velocity (rot/s). Positive = inward. */
  public default void setLeftShaftVelocity(double velocityRotPerSec) {}

  /** Run right mecanum shaft at velocity (rot/s). Positive = inward. */
  public default void setRightShaftVelocity(double velocityRotPerSec) {}

  /** Stop all motors. */
  public default void stop() {}
}
