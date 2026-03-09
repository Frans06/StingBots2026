package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor EXTENSION_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim extensionSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(EXTENSION_GEARBOX, 0.004, 1.0), EXTENSION_GEARBOX);
  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, 0.004, 1.0), ROLLER_GEARBOX);

  private final PIDController extensionController =
      new PIDController(IntakeConstants.EXTENSION_KP, 0, IntakeConstants.EXTENSION_KD);
  private final PIDController rollerController =
      new PIDController(IntakeConstants.ROLLER_KP, 0, IntakeConstants.ROLLER_KD);

  private boolean extensionClosedLoop = false;
  private boolean rollerClosedLoop = false;
  private double extensionAppliedVolts = 0.0;
  private double rollerAppliedVolts = 0.0;
  private double extensionFFVolts = 0.0;
  private double rollerFFVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (extensionClosedLoop) {
      extensionAppliedVolts =
          extensionFFVolts + extensionController.calculate(extensionSim.getAngularPositionRad());
    }
    if (rollerClosedLoop) {
      rollerAppliedVolts =
          rollerFFVolts + rollerController.calculate(rollerSim.getAngularVelocityRadPerSec());
    }

    extensionSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
    extensionSim.update(0.02);
    rollerSim.update(0.02);

    inputs.extensionConnected = true;
    inputs.extensionPositionRot = extensionSim.getAngularPositionRotations();
    inputs.extensionVelocityRotPerSec = extensionSim.getAngularVelocityRPM() / 60.0;
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionCurrentAmps = Math.abs(extensionSim.getCurrentDrawAmps());

    inputs.rollerConnected = true;
    inputs.rollerVelocityRotPerSec = rollerSim.getAngularVelocityRPM() / 60.0;
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
  }

  @Override
  public void setExtensionPosition(double positionRot) {
    extensionClosedLoop = true;
    double positionRad = positionRot * 2.0 * Math.PI;
    extensionFFVolts = IntakeConstants.EXTENSION_KS * Math.signum(positionRad);
    extensionController.setSetpoint(positionRad);
  }

  @Override
  public void setRollerVelocity(double velocityRotPerSec) {
    rollerClosedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    rollerFFVolts =
        IntakeConstants.ROLLER_KS * Math.signum(velocityRadPerSec)
            + IntakeConstants.ROLLER_KV * velocityRadPerSec;
    rollerController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stop() {
    extensionClosedLoop = false;
    rollerClosedLoop = false;
    extensionAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
  }
}
