package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.FeederConstants;

public class FeederIOSim implements FeederIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim kickerSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.004, 1.0), GEARBOX);

  private final PIDController kickerController = new PIDController(FeederConstants.KP, 0, 0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = ffVolts + kickerController.calculate(kickerSim.getAngularVelocityRadPerSec());
    }

    kickerSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    kickerSim.update(0.02);

    inputs.kickerConnected = true;
    inputs.kickerVelocityRotPerSec = kickerSim.getAngularVelocityRPM() / 60.0;
    inputs.kickerAppliedVolts = appliedVolts;
    inputs.kickerCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
  }

  @Override
  public void setKickerVelocity(double velocityRotPerSec) {
    closedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    ffVolts =
        FeederConstants.KS * Math.signum(velocityRadPerSec)
            + FeederConstants.KV * velocityRadPerSec;
    kickerController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
