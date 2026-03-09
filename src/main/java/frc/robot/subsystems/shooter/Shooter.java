package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final String name;

  private double targetHoodPositionRot = ShooterConstants.HOOD_DOWN_POS;
  private double targetFlywheelVelocity = 0.0;

  private final Alert hoodDisconnected;
  private final Alert flywheelDisconnected;

  public Shooter(ShooterIO io, String name) {
    this.io = io;
    this.name = name;
    hoodDisconnected = new Alert(name + " hood motor disconnected.", AlertType.kError);
    flywheelDisconnected = new Alert(name + " flywheel motor disconnected.", AlertType.kError);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Log target setpoints for PID tuning (not part of IO inputs)
    Logger.recordOutput(name + "/TargetHoodPositionRot", targetHoodPositionRot);
    Logger.recordOutput(name + "/TargetFlywheelVelocity", targetFlywheelVelocity);

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    hoodDisconnected.set(!inputs.hoodConnected);
    flywheelDisconnected.set(!inputs.flywheelConnected);
  }

  /** Spin up the shooter: hood to 45 degrees (6 rot) and flywheel to default velocity. */
  public void spinUp() {
    targetHoodPositionRot = ShooterConstants.HOOD_UP_POS;
    targetFlywheelVelocity = ShooterConstants.FLYWHEEL_DEFAULT_VELOCITY;
    io.setHoodPosition(targetHoodPositionRot);
    io.setFlywheelVelocity(targetFlywheelVelocity);
  }

  /** Spin up with specific parameters. */
  public void spinUp(double hoodPositionRot, double flywheelVelocity) {
    targetHoodPositionRot = hoodPositionRot;
    targetFlywheelVelocity = flywheelVelocity;
    io.setHoodPosition(targetHoodPositionRot);
    io.setFlywheelVelocity(targetFlywheelVelocity);
  }

  public void stop() {
    targetFlywheelVelocity = 0.0;
    io.stop();
  }

  @AutoLogOutput
  public boolean isHoodAtTarget() {
    return Math.abs(inputs.hoodPositionRot - targetHoodPositionRot)
        < ShooterConstants.HOOD_POSITION_TOLERANCE;
  }

  @AutoLogOutput
  public boolean isFlywheelAtTarget() {
    return targetFlywheelVelocity > 0.0
        && Math.abs(inputs.flywheelVelocityRotPerSec - targetFlywheelVelocity)
            < ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
  }

  @AutoLogOutput
  public boolean isReadyToFire() {
    return isHoodAtTarget() && isFlywheelAtTarget();
  }

  public double getFlywheelVelocity() {
    return inputs.flywheelVelocityRotPerSec;
  }

  public double getHoodPosition() {
    return inputs.hoodPositionRot;
  }
}
