package frc.robot.subsystems.feeder;

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
import frc.robot.Constants.FeederConstants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX kickerTalon;

  private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> kickerVelocity;
  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerCurrent;

  private final Debouncer kickerDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FeederIOTalonFX() {
    kickerTalon = new TalonFX(FeederConstants.KICKER_MOTOR_ID, Constants.CAN_BUS);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = FeederConstants.KP;
    config.Slot0.kI = FeederConstants.KI;
    config.Slot0.kD = FeederConstants.KD;
    config.Slot0.kS = FeederConstants.KS;
    config.Slot0.kV = FeederConstants.KV;
    config.CurrentLimits.StatorCurrentLimit = FeederConstants.STATOR_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> kickerTalon.getConfigurator().apply(config, 0.25));

    kickerVelocity = kickerTalon.getVelocity();
    kickerAppliedVolts = kickerTalon.getMotorVoltage();
    kickerCurrent = kickerTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, kickerVelocity, kickerAppliedVolts, kickerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(kickerTalon);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(kickerVelocity, kickerAppliedVolts, kickerCurrent);

    inputs.kickerConnected = kickerDebounce.calculate(status.isOK());
    inputs.kickerVelocityRotPerSec = kickerVelocity.getValueAsDouble();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrent.getValueAsDouble();
  }

  @Override
  public void setKickerVelocity(double velocityRotPerSec) {
    kickerTalon.setControl(kickerVelocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    kickerTalon.stopMotor();
  }
}
