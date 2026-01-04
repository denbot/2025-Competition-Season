package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.CanBeAnInstrument;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class BoathookIOTalonFX implements BoathookIO, CanBeAnInstrument {
  private final TalonFX rotationMotor = new TalonFX(
      Constants.BoathookConstants.ROTATION_MOTOR_ID,
      Constants.OperatorConstants.canivoreSerial
  );

  private final TalonFX extensionMotor = new TalonFX(
      Constants.BoathookConstants.EXTENDER_MOTOR_ID,
      Constants.OperatorConstants.canivoreSerial
  );

  private final CANcoder rotationEncoder = new CANcoder(
      Constants.BoathookConstants.ROTATION_ENCODER_ID,
      Constants.OperatorConstants.canivoreSerial
  );

  private final CANcoder extensionEncoder = new CANcoder(
      Constants.BoathookConstants.EXTENDER_ENCODER_ID,
      Constants.OperatorConstants.canivoreSerial
  );

  private final CANdi limitSensors = new CANdi(  // Do we need to add limitSensor to the input interface?
      Constants.BoathookConstants.CANDI_ID,
      Constants.OperatorConstants.canivoreSerial
  );

  // Status Signals for boathook
  private final StatusSignal<Angle> rotationAngle = rotationMotor.getPosition();
  private final StatusSignal<AngularVelocity> rotationAngularVelocity = rotationMotor.getVelocity();
  private final StatusSignal<Voltage> rotationAppliedVolts = rotationMotor.getMotorVoltage();
  private final StatusSignal<Current> rotationCurrent = rotationMotor.getStatorCurrent();

  private final Distance wheelCircumference = Inches.of(1.89).times(2 * Math.PI);
  private final StatusSignal<Angle> extensionAngle = extensionMotor.getPosition();
  private final StatusSignal<AngularVelocity> extensionAngularVelocity = extensionMotor.getVelocity();
  private final StatusSignal<Voltage> extensionAppliedVolts = extensionMotor.getMotorVoltage();
  private final StatusSignal<Current> extensionCurrent = extensionMotor.getStatorCurrent();

  // Connection debouncers
  private final Debouncer rotationConnectedDebounce = new Debouncer(0.5);
  private final Debouncer extensionConnectedDebounce = new Debouncer(0.5);

  public BoathookIOTalonFX() {
    var rotationConfig = new TalonFXConfiguration();
    rotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.StatorCurrentLimit = 80;  // TODO BoathookConstants
    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.Feedback.FeedbackRemoteSensorID = Constants.BoathookConstants.ROTATION_ENCODER_ID;
    rotationConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    rotationConfig.Feedback.RotorToSensorRatio = Constants.BoathookConstants.ROTATOR_GEAR_RATIO;
    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.BoathookConstants.ROTATOR_FORWARD_LIMIT;
    rotationConfig.Slot0.kP = 20;  // TODO BoathookConstants
    rotationConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    rotationConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    rotationConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    rotationConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    rotationConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = Constants.BoathookConstants.CANDI_ID;
    rotationConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.BoathookConstants.CANDI_ID;
    rotationConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
    rotationConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    rotationConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    rotationConfig.Audio.AllowMusicDurDisable = true;

    tryUntilOk(5, () -> rotationMotor.getConfigurator().apply(rotationConfig, 0.25));

    var extenderConfig = new TalonFXConfiguration();
    extenderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extenderConfig.CurrentLimits.StatorCurrentLimit = 80;
    extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extenderConfig.Feedback.FeedbackRemoteSensorID = Constants.BoathookConstants.EXTENDER_ENCODER_ID;
    extenderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    extenderConfig.Feedback.RotorToSensorRatio = 25 * (34.0 / 24.0);  // TODO BoathookConstants
    extenderConfig.Feedback.SensorToMechanismRatio = 1.8;  // TODO BoathookConstants
    extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extenderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.BoathookConstants.EXTENDER_FORWARD_LIMIT;
    extenderConfig.Slot0.kP = 20;  // TODO BoathookConstants
    extenderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extenderConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    extenderConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    extenderConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = Constants.BoathookConstants.EXTENDER_REVERSE_LIMIT;
    extenderConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.BoathookConstants.CANDI_ID;
    extenderConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;
    extenderConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    extenderConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    extenderConfig.Audio.AllowMusicDurDisable = true;

    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extenderConfig, 0.25));

    CANcoderConfiguration rotationEncoderConfig = new CANcoderConfiguration();
    rotationEncoderConfig.MagnetSensor.MagnetOffset = -0.493431640625;  // TODO BoathookConstants
    rotationEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> rotationEncoder.getConfigurator().apply(rotationEncoderConfig, 0.25));

    CANcoderConfiguration extensionEncoderConfig = new CANcoderConfiguration();
    rotationEncoderConfig.MagnetSensor.MagnetOffset = 0.15312765625;  // TODO BoathookConstants
    rotationEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    tryUntilOk(5, () -> extensionEncoder.getConfigurator().apply(extensionEncoderConfig, 0.25));

    CANdiConfiguration limitSensorsConfig = new CANdiConfiguration();
    limitSensorsConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    limitSensorsConfig.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect;
    limitSensorsConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
    limitSensorsConfig.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;

    tryUntilOk(5, () -> limitSensors.getConfigurator().apply(limitSensorsConfig, 0.25));

    // Tell all of our signals to start updating at the correct frequency depending on licensure
    BaseStatusSignal.setUpdateFrequencyForAll(
        rotationMotor.getIsProLicensed().getValue() ? 200 : 50,
        rotationAngle,
        rotationAngularVelocity,
        rotationAppliedVolts,
        rotationCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        extensionMotor.getIsProLicensed().getValue() ? 200 : 50,
        extensionAngle,
        extensionAngularVelocity,
        extensionAppliedVolts,
        extensionCurrent);

    ParentDevice.optimizeBusUtilizationForAll(rotationMotor, extensionMotor);
  }

  @Override
  public void updateInputs(BoathookIOInputs inputs) {
    var rotationStatus = BaseStatusSignal.refreshAll(rotationAngle, rotationAngularVelocity, rotationAppliedVolts, rotationCurrent);
    var extensionStatus = BaseStatusSignal.refreshAll(extensionAngle, extensionAngularVelocity, extensionAppliedVolts, extensionCurrent);

    inputs.rotationConnected = rotationConnectedDebounce.calculate(rotationStatus.isOK());
    inputs.angle = rotationAngle.getValue();
    inputs.angularVelocity = rotationAngularVelocity.getValue();
    inputs.angleAppliedVolts = rotationAppliedVolts.getValue();
    inputs.angleCurrent = rotationCurrent.getValue();

    inputs.extensionConnected = extensionConnectedDebounce.calculate(extensionStatus.isOK());
    inputs.extensionLength = wheelCircumference.times(extensionAngle.getValue().in(Rotations));
    inputs.extensionLengthPerSec = wheelCircumference.times(extensionAngularVelocity.getValue().in(RotationsPerSecond)).per(Second);
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValue();
    inputs.extensionCurrentAmps = extensionCurrent.getValue();
  }

  @Override
  public void setAngleVoltage(double voltage) {
    rotationMotor.setVoltage(voltage);
  }

  @Override
  public void setAnglePosition(Angle angle) {
    rotationMotor.setControl(new PositionVoltage(angle));
  }

  @Override
  public void setLengthVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }

  @Override
  public void setLengthPosition(Distance length) {
    double motorAngleRotations = length.div(wheelCircumference).in(Value);
    extensionMotor.setControl(new PositionVoltage(motorAngleRotations));
  }

  @Override
  public void addInstruments(Orchestra orchestra) {
    orchestra.addInstrument(rotationMotor);
    orchestra.addInstrument(extensionMotor);
  }
}
