package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim hoodSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, 0.004, 1.0), HOOD_GEARBOX);
  private final DCMotorSim flywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(FLYWHEEL_GEARBOX, 0.025, 1.0), FLYWHEEL_GEARBOX);

  private final PIDController hoodController =
      new PIDController(ShooterConstants.HOOD_KP, 0, ShooterConstants.HOOD_KD);
  private final PIDController flywheelController =
      new PIDController(ShooterConstants.FLYWHEEL_KP, 0, 0);

  private boolean hoodClosedLoop = false;
  private boolean flywheelClosedLoop = false;
  private double hoodAppliedVolts = 0.0;
  private double flywheelAppliedVolts = 0.0;
  private double hoodFFVolts = 0.0;
  private double flywheelFFVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (hoodClosedLoop) {
      hoodAppliedVolts = hoodFFVolts + hoodController.calculate(hoodSim.getAngularPositionRad());
    }
    if (flywheelClosedLoop) {
      flywheelAppliedVolts =
          flywheelFFVolts + flywheelController.calculate(flywheelSim.getAngularVelocityRadPerSec());
    }

    hoodSim.setInputVoltage(MathUtil.clamp(hoodAppliedVolts, -12.0, 12.0));
    flywheelSim.setInputVoltage(MathUtil.clamp(flywheelAppliedVolts, -12.0, 12.0));
    hoodSim.update(0.02);
    flywheelSim.update(0.02);

    inputs.hoodConnected = true;
    inputs.hoodPositionRot = hoodSim.getAngularPositionRotations();
    inputs.hoodVelocityRotPerSec = hoodSim.getAngularVelocityRPM() / 60.0;
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());

    inputs.flywheelConnected = true;
    inputs.flywheelVelocityRotPerSec = flywheelSim.getAngularVelocityRPM() / 60.0;
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setHoodPosition(double positionRot) {
    hoodClosedLoop = true;
    double positionRad = positionRot * 2.0 * Math.PI;
    hoodFFVolts = ShooterConstants.HOOD_KS * Math.signum(positionRad);
    hoodController.setSetpoint(positionRad);
  }

  @Override
  public void setFlywheelVelocity(double velocityRotPerSec) {
    flywheelClosedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    flywheelFFVolts =
        ShooterConstants.FLYWHEEL_KS * Math.signum(velocityRadPerSec)
            + ShooterConstants.FLYWHEEL_KV * velocityRadPerSec;
    flywheelController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stop() {
    hoodClosedLoop = false;
    flywheelClosedLoop = false;
    hoodAppliedVolts = 0.0;
    flywheelAppliedVolts = 0.0;
  }
}
