package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim feedSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.004, 1.0), GEARBOX);
  private final DCMotorSim leftShaftSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.004, 1.0), GEARBOX);
  private final DCMotorSim rightShaftSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.004, 1.0), GEARBOX);

  private final PIDController feedController = new PIDController(IndexerConstants.KP, 0, 0);
  private final PIDController leftController = new PIDController(IndexerConstants.KP, 0, 0);
  private final PIDController rightController = new PIDController(IndexerConstants.KP, 0, 0);

  private boolean feedClosedLoop = false;
  private boolean leftClosedLoop = false;
  private boolean rightClosedLoop = false;
  private double feedAppliedVolts = 0.0;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double feedFFVolts = 0.0;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (feedClosedLoop) {
      feedAppliedVolts =
          feedFFVolts + feedController.calculate(feedSim.getAngularVelocityRadPerSec());
    }
    if (leftClosedLoop) {
      leftAppliedVolts =
          leftFFVolts + leftController.calculate(leftShaftSim.getAngularVelocityRadPerSec());
    }
    if (rightClosedLoop) {
      rightAppliedVolts =
          rightFFVolts + rightController.calculate(rightShaftSim.getAngularVelocityRadPerSec());
    }

    feedSim.setInputVoltage(MathUtil.clamp(feedAppliedVolts, -12.0, 12.0));
    leftShaftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
    rightShaftSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    feedSim.update(0.02);
    leftShaftSim.update(0.02);
    rightShaftSim.update(0.02);

    inputs.feedConnected = true;
    inputs.feedVelocityRotPerSec = feedSim.getAngularVelocityRPM() / 60.0;
    inputs.feedAppliedVolts = feedAppliedVolts;
    inputs.feedCurrentAmps = Math.abs(feedSim.getCurrentDrawAmps());

    inputs.leftShaftConnected = true;
    inputs.leftShaftVelocityRotPerSec = leftShaftSim.getAngularVelocityRPM() / 60.0;
    inputs.leftShaftAppliedVolts = leftAppliedVolts;
    inputs.leftShaftCurrentAmps = Math.abs(leftShaftSim.getCurrentDrawAmps());

    inputs.rightShaftConnected = true;
    inputs.rightShaftVelocityRotPerSec = rightShaftSim.getAngularVelocityRPM() / 60.0;
    inputs.rightShaftAppliedVolts = rightAppliedVolts;
    inputs.rightShaftCurrentAmps = Math.abs(rightShaftSim.getCurrentDrawAmps());
  }

  @Override
  public void setFeedVelocity(double velocityRotPerSec) {
    feedClosedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    feedFFVolts =
        IndexerConstants.KS * Math.signum(velocityRadPerSec)
            + IndexerConstants.KV * velocityRadPerSec;
    feedController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setLeftShaftVelocity(double velocityRotPerSec) {
    leftClosedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    leftFFVolts =
        IndexerConstants.KS * Math.signum(velocityRadPerSec)
            + IndexerConstants.KV * velocityRadPerSec;
    leftController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setRightShaftVelocity(double velocityRotPerSec) {
    rightClosedLoop = true;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    rightFFVolts =
        IndexerConstants.KS * Math.signum(velocityRadPerSec)
            + IndexerConstants.KV * velocityRadPerSec;
    rightController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stop() {
    feedClosedLoop = false;
    leftClosedLoop = false;
    rightClosedLoop = false;
    feedAppliedVolts = 0.0;
    leftAppliedVolts = 0.0;
    rightAppliedVolts = 0.0;
  }
}
