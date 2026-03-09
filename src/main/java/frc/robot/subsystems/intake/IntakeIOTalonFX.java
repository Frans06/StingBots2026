package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX extensionTalon;
  private final TalonFX rollerTalon;

  private final MotionMagicVoltage extensionMotionMagic = new MotionMagicVoltage(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> extensionPosition;
  private final StatusSignal<AngularVelocity> extensionVelocity;
  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Current> extensionCurrent;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  private final Debouncer extensionDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    extensionTalon = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID, Constants.CAN_BUS);
    rollerTalon = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, Constants.CAN_BUS);

    // Configure extension motor
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionConfig.Slot0.kP = IntakeConstants.EXTENSION_KP;
    extensionConfig.Slot0.kI = IntakeConstants.EXTENSION_KI;
    extensionConfig.Slot0.kD = IntakeConstants.EXTENSION_KD;
    extensionConfig.Slot0.kS = IntakeConstants.EXTENSION_KS;
    extensionConfig.Slot0.kV = IntakeConstants.EXTENSION_KV;
    extensionConfig.Slot0.kA = IntakeConstants.EXTENSION_KA;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.EXTENSION_CRUISE_VELOCITY;
    extensionConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.EXTENSION_ACCELERATION;
    extensionConfig.MotionMagic.MotionMagicJerk = IntakeConstants.EXTENSION_JERK;
    extensionConfig.Feedback.SensorToMechanismRatio = IntakeConstants.EXTENSION_GEAR_RATIO;
    extensionConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.EXTENSION_STATOR_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.EXTENSION_EXTENDED_POS + 0.5;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    tryUntilOk(5, () -> extensionTalon.getConfigurator().apply(extensionConfig, 0.25));
    tryUntilOk(5, () -> extensionTalon.setPosition(0.0, 0.25));

    // Configure roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.Slot0.kP = IntakeConstants.ROLLER_KP;
    rollerConfig.Slot0.kI = IntakeConstants.ROLLER_KI;
    rollerConfig.Slot0.kD = IntakeConstants.ROLLER_KD;
    rollerConfig.Slot0.kS = IntakeConstants.ROLLER_KS;
    rollerConfig.Slot0.kV = IntakeConstants.ROLLER_KV;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerConfig, 0.25));

    // Create status signals
    extensionPosition = extensionTalon.getPosition();
    extensionVelocity = extensionTalon.getVelocity();
    extensionAppliedVolts = extensionTalon.getMotorVoltage();
    extensionCurrent = extensionTalon.getStatorCurrent();

    rollerVelocity = rollerTalon.getVelocity();
    rollerAppliedVolts = rollerTalon.getMotorVoltage();
    rollerCurrent = rollerTalon.getStatorCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionPosition,
        extensionVelocity,
        extensionAppliedVolts,
        extensionCurrent,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(extensionTalon, rollerTalon);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var extStatus =
        BaseStatusSignal.refreshAll(
            extensionPosition, extensionVelocity, extensionAppliedVolts, extensionCurrent);
    var rolStatus = BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);

    inputs.extensionConnected = extensionDebounce.calculate(extStatus.isOK());
    inputs.extensionPositionRot = extensionPosition.getValueAsDouble();
    inputs.extensionVelocityRotPerSec = extensionVelocity.getValueAsDouble();
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionCurrentAmps = extensionCurrent.getValueAsDouble();

    inputs.rollerConnected = rollerDebounce.calculate(rolStatus.isOK());
    inputs.rollerVelocityRotPerSec = rollerVelocity.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
  }

  @Override
  public void setExtensionPosition(double positionRot) {
    extensionTalon.setControl(extensionMotionMagic.withPosition(positionRot));
  }

  @Override
  public void setRollerVelocity(double velocityRotPerSec) {
    rollerTalon.setControl(rollerVelocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void stop() {
    extensionTalon.stopMotor();
    rollerTalon.stopMotor();
  }
}
