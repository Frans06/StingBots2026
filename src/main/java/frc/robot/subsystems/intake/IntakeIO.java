package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean extensionConnected = false;
    public double extensionPositionRot = 0.0;
    public double extensionVelocityRotPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;

    public boolean rollerConnected = false;
    public double rollerVelocityRotPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set extension to target position using MotionMagic (rotations). */
  public default void setExtensionPosition(double positionRot) {}

  /** Run roller at target velocity (rot/s). */
  public default void setRollerVelocity(double velocityRotPerSec) {}

  /** Stop all motors. */
  public default void stop() {}
}
