package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert extensionDisconnected =
      new Alert("Intake extension motor disconnected.", AlertType.kError);
  private final Alert rollerDisconnected =
      new Alert("Intake roller motor disconnected.", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    extensionDisconnected.set(!inputs.extensionConnected);
    rollerDisconnected.set(!inputs.rollerConnected);
  }

  /** Extend the intake and start rollers for intaking. */
  public void extend() {
    io.setExtensionPosition(IntakeConstants.EXTENSION_EXTENDED_POS);
    io.setRollerVelocity(IntakeConstants.ROLLER_INTAKE_VELOCITY);
  }

  /** Retract the intake and stop rollers. */
  public void retract() {
    io.setExtensionPosition(IntakeConstants.EXTENSION_RETRACTED_POS);
    io.stop();
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Intake/IsRetracted")
  public boolean isRetracted() {
    return Math.abs(inputs.extensionPositionRot - IntakeConstants.EXTENSION_RETRACTED_POS)
        < IntakeConstants.EXTENSION_POSITION_TOLERANCE;
  }

  @AutoLogOutput(key = "Intake/IsExtended")
  public boolean isExtended() {
    return Math.abs(inputs.extensionPositionRot - IntakeConstants.EXTENSION_EXTENDED_POS)
        < IntakeConstants.EXTENSION_POSITION_TOLERANCE;
  }

  public double getExtensionPosition() {
    return inputs.extensionPositionRot;
  }

  public double getRollerVelocity() {
    return inputs.rollerVelocityRotPerSec;
  }
}
