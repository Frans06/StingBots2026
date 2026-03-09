package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX hoodTalon;
  private final TalonFX flywheelTalon;

  private final MotionMagicVoltage hoodMotionMagic = new MotionMagicVoltage(0.0);
  private final MotionMagicVelocityVoltage flywheelMotionMagicVelocity =
      new MotionMagicVelocityVoltage(0.0);

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;

  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;

  private final Debouncer hoodDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer flywheelDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX(int hoodMotorId, int flywheelMotorId) {
    hoodTalon = new TalonFX(hoodMotorId, Constants.CAN_BUS);
    flywheelTalon = new TalonFX(flywheelMotorId, Constants.CAN_BUS);

    // Configure hood motor (MotionMagic position, Kraken X44)
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.Slot0.kP = ShooterConstants.HOOD_KP;
    hoodConfig.Slot0.kI = ShooterConstants.HOOD_KI;
    hoodConfig.Slot0.kD = ShooterConstants.HOOD_KD;
    hoodConfig.Slot0.kS = ShooterConstants.HOOD_KS;
    hoodConfig.Slot0.kV = ShooterConstants.HOOD_KV;
    hoodConfig.Slot0.kA = ShooterConstants.HOOD_KA;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.HOOD_CRUISE_VELOCITY;
    hoodConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.HOOD_ACCELERATION;
    hoodConfig.MotionMagic.MotionMagicJerk = ShooterConstants.HOOD_JERK;
    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;
    hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.HOOD_STATOR_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.HOOD_UP_POS + 0.5;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.HOOD_DOWN_POS - 0.5;
    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(hoodConfig, 0.25));
    tryUntilOk(5, () -> hoodTalon.setPosition(0.0, 0.25));

    // Configure flywheel motor (MotionMagicVelocity, Kraken X60)
    var flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_KP;
    flywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_KI;
    flywheelConfig.Slot0.kD = ShooterConstants.FLYWHEEL_KD;
    flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_KS;
    flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_KV;
    flywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_KA;
    flywheelConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.FLYWHEEL_ACCELERATION;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> flywheelTalon.getConfigurator().apply(flywheelConfig, 0.25));

    // Create status signals
    hoodPosition = hoodTalon.getPosition();
    hoodVelocity = hoodTalon.getVelocity();
    hoodAppliedVolts = hoodTalon.getMotorVoltage();
    hoodCurrent = hoodTalon.getStatorCurrent();

    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
    flywheelCurrent = flywheelTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent);
    ParentDevice.optimizeBusUtilizationForAll(hoodTalon, flywheelTalon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var hoodStatus =
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);
    var fwStatus =
        BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);

    inputs.hoodConnected = hoodDebounce.calculate(hoodStatus.isOK());
    inputs.hoodPositionRot = hoodPosition.getValueAsDouble();
    inputs.hoodVelocityRotPerSec = hoodVelocity.getValueAsDouble();
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    inputs.flywheelConnected = flywheelDebounce.calculate(fwStatus.isOK());
    inputs.flywheelVelocityRotPerSec = flywheelVelocity.getValueAsDouble();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelCurrent.getValueAsDouble();
  }

  @Override
  public void setHoodPosition(double positionRot) {
    hoodTalon.setControl(hoodMotionMagic.withPosition(positionRot));
  }

  @Override
  public void setFlywheelVelocity(double velocityRotPerSec) {
    flywheelTalon.setControl(flywheelMotionMagicVelocity.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    hoodTalon.stopMotor();
    flywheelTalon.stopMotor();
  }
}
