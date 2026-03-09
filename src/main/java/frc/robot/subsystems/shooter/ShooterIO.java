package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean hoodConnected = false;
    public double hoodPositionRot = 0.0;
    public double hoodVelocityRotPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;

    public boolean flywheelConnected = false;
    public double flywheelVelocityRotPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set hood to target position using MotionMagic (rotations). */
  public default void setHoodPosition(double positionRot) {}

  /** Set flywheel speed using MotionMagicVelocity (rot/s). */
  public default void setFlywheelVelocity(double velocityRotPerSec) {}

  /** Stop all motors. */
  public default void stop() {}
}
