package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX feedTalon;
  private final TalonFX leftShaftTalon;
  private final TalonFX rightShaftTalon;

  private final VelocityVoltage feedVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage leftShaftVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage rightShaftVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> feedVelocity;
  private final StatusSignal<Voltage> feedAppliedVolts;
  private final StatusSignal<Current> feedCurrent;

  private final StatusSignal<AngularVelocity> leftShaftVelocity;
  private final StatusSignal<Voltage> leftShaftAppliedVolts;
  private final StatusSignal<Current> leftShaftCurrent;

  private final StatusSignal<AngularVelocity> rightShaftVelocity;
  private final StatusSignal<Voltage> rightShaftAppliedVolts;
  private final StatusSignal<Current> rightShaftCurrent;

  private final Debouncer feedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer leftShaftDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightShaftDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IndexerIOTalonFX() {
    feedTalon = new TalonFX(IndexerConstants.FEED_MOTOR_ID, Constants.CAN_BUS);
    leftShaftTalon = new TalonFX(IndexerConstants.LEFT_SHAFT_MOTOR_ID, Constants.CAN_BUS);
    rightShaftTalon = new TalonFX(IndexerConstants.RIGHT_SHAFT_MOTOR_ID, Constants.CAN_BUS);

    // Configure all three motors with shared gains
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = IndexerConstants.KP;
    config.Slot0.kI = IndexerConstants.KI;
    config.Slot0.kD = IndexerConstants.KD;
    config.Slot0.kS = IndexerConstants.KS;
    config.Slot0.kV = IndexerConstants.KV;
    config.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> feedTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leftShaftTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightShaftTalon.getConfigurator().apply(config, 0.25));

    // Create status signals
    feedVelocity = feedTalon.getVelocity();
    feedAppliedVolts = feedTalon.getMotorVoltage();
    feedCurrent = feedTalon.getStatorCurrent();

    leftShaftVelocity = leftShaftTalon.getVelocity();
    leftShaftAppliedVolts = leftShaftTalon.getMotorVoltage();
    leftShaftCurrent = leftShaftTalon.getStatorCurrent();

    rightShaftVelocity = rightShaftTalon.getVelocity();
    rightShaftAppliedVolts = rightShaftTalon.getMotorVoltage();
    rightShaftCurrent = rightShaftTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        feedVelocity,
        feedAppliedVolts,
        feedCurrent,
        leftShaftVelocity,
        leftShaftAppliedVolts,
        leftShaftCurrent,
        rightShaftVelocity,
        rightShaftAppliedVolts,
        rightShaftCurrent);
    ParentDevice.optimizeBusUtilizationForAll(feedTalon, leftShaftTalon, rightShaftTalon);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var feedStatus = BaseStatusSignal.refreshAll(feedVelocity, feedAppliedVolts, feedCurrent);
    var leftStatus =
        BaseStatusSignal.refreshAll(leftShaftVelocity, leftShaftAppliedVolts, leftShaftCurrent);
    var rightStatus =
        BaseStatusSignal.refreshAll(rightShaftVelocity, rightShaftAppliedVolts, rightShaftCurrent);

    inputs.feedConnected = feedDebounce.calculate(feedStatus.isOK());
    inputs.feedVelocityRotPerSec = feedVelocity.getValueAsDouble();
    inputs.feedAppliedVolts = feedAppliedVolts.getValueAsDouble();
    inputs.feedCurrentAmps = feedCurrent.getValueAsDouble();

    inputs.leftShaftConnected = leftShaftDebounce.calculate(leftStatus.isOK());
    inputs.leftShaftVelocityRotPerSec = leftShaftVelocity.getValueAsDouble();
    inputs.leftShaftAppliedVolts = leftShaftAppliedVolts.getValueAsDouble();
    inputs.leftShaftCurrentAmps = leftShaftCurrent.getValueAsDouble();

    inputs.rightShaftConnected = rightShaftDebounce.calculate(rightStatus.isOK());
    inputs.rightShaftVelocityRotPerSec = rightShaftVelocity.getValueAsDouble();
    inputs.rightShaftAppliedVolts = rightShaftAppliedVolts.getValueAsDouble();
    inputs.rightShaftCurrentAmps = rightShaftCurrent.getValueAsDouble();
  }

  @Override
  public void setFeedVelocity(double velocityRotPerSec) {
    feedTalon.setControl(feedVelocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setLeftShaftVelocity(double velocityRotPerSec) {
    leftShaftTalon.setControl(leftShaftVelocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setRightShaftVelocity(double velocityRotPerSec) {
    rightShaftTalon.setControl(rightShaftVelocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    feedTalon.stopMotor();
    leftShaftTalon.stopMotor();
    rightShaftTalon.stopMotor();
  }
}
