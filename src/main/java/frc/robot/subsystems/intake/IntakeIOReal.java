
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.OperatorConstants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX intakeLeft =
      new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX intakeRight =
      new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX rotator =
      new TalonFX(IntakeConstants.INTAKE_ROTATOR_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotatorEncoder =
      new CANcoder(IntakeConstants.INTAKE_ROTATOR_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANdi intakeSensors =
      new CANdi(IntakeConstants.CANDI_ID, OperatorConstants.canivoreSerial);

  private static final VelocityTorqueCurrentFOC intakeSpin =
      new VelocityTorqueCurrentFOC(0).withAcceleration(IntakeConstants.intakeAcceleration);

  // Connection debouncers
  private final Debouncer intakeLeftConnectedDebounce = new Debouncer(0.5);
  private final Debouncer intakeRightConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rotatorConnectedDebounce = new Debouncer(0.5);

  private final StatusSignal<AngularVelocity> leftVelocityRotPerSec = intakeLeft.getVelocity();
  private final StatusSignal<Current> leftCurrentAmps = intakeLeft.getSupplyCurrent();

  private final StatusSignal<AngularVelocity> rightVelocityRotPerSec = intakeRight.getVelocity();
  private final StatusSignal<Current> rightCurrentAmps = intakeRight.getSupplyCurrent();

  private final StatusSignal<Angle> rotatorPositionRot = rotator.getPosition();
  private final StatusSignal<Double> rotatorClosedLoopErrorRot = rotator.getClosedLoopError();
  private final StatusSignal<AngularVelocity> rotatorVelocityRotPerSec = rotator.getVelocity();

  public IntakeIOReal() {
    var intakeRotatorConfig =
    new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(IntakeConstants.INTAKE_ROTATOR_ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(IntakeConstants.rotatorGearRatio)
                .withRotorToSensorRatio(120))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withForwardSoftLimitThreshold(IntakeConstants.forwardSoftLimit)
                .withReverseSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(IntakeConstants.reverseSoftLimit))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(4)
                .withMotionMagicCruiseVelocity(2))
        .withSlot0(
            new Slot0Configs()
                .withKP(45)
                .withKD(0)
                .withKG(0.2)
                .withGravityType(GravityTypeValue.Arm_Cosine));

    var intakeRotatorSensorConfig =
    new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withMagnetOffset(-0.907070703125)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    var intakeSensorsConfig =
    new CANdiConfiguration()
        .withDigitalInputs(
            new DigitalInputsConfigs()
                .withS1CloseState(S1CloseStateValue.CloseWhenNotFloating)
                .withS1FloatState(S1FloatStateValue.FloatDetect)
                .withS2CloseState(S2CloseStateValue.CloseWhenNotFloating)
                .withS2FloatState(S2FloatStateValue.FloatDetect));

    var intakeConfig =
    new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60))
        .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

    rotator.setNeutralMode(NeutralModeValue.Brake);
    intakeLeft.setNeutralMode(NeutralModeValue.Coast);
    intakeRight.setNeutralMode(NeutralModeValue.Coast);

    tryUntilOk(5, () -> rotator.getConfigurator().apply(intakeRotatorConfig, 0.25));
    tryUntilOk(5, () -> rotatorEncoder.getConfigurator().apply(intakeRotatorSensorConfig, 0.25));
    tryUntilOk(5, () -> intakeLeft.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> intakeRight.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> intakeSensors.getConfigurator().apply(intakeSensorsConfig,0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        intakeLeft.getIsProLicensed().getValue() ? 200 : 50, 
        leftVelocityRotPerSec, 
        leftCurrentAmps
    );
    BaseStatusSignal.setUpdateFrequencyForAll(
        intakeRight.getIsProLicensed().getValue() ? 200 : 50,
        rightVelocityRotPerSec,
        rightCurrentAmps
    );
    BaseStatusSignal.setUpdateFrequencyForAll(
        rotator.getIsProLicensed().getValue() ? 200 : 50,
        rotatorPositionRot,
        rotatorClosedLoopErrorRot,
        rotatorVelocityRotPerSec
    );

    ParentDevice.optimizeBusUtilizationForAll(intakeLeft,intakeRight,rotator);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeLeftStatus = BaseStatusSignal.refreshAll(leftVelocityRotPerSec, leftCurrentAmps);
    var intakeRightStatus = BaseStatusSignal.refreshAll(rightVelocityRotPerSec, rightCurrentAmps);
    var rotatorStatus = BaseStatusSignal.refreshAll(rotatorPositionRot, rotatorClosedLoopErrorRot, rotatorVelocityRotPerSec);

    inputs.intakeLeftConnected = intakeLeftConnectedDebounce.calculate(intakeLeftStatus.isOK());
    inputs.intakeRightConnected = intakeRightConnectedDebounce.calculate(intakeRightStatus.isOK());
    inputs.rotatorConnected = rotatorConnectedDebounce.calculate(rotatorStatus.isOK());

    inputs.leftVelocityRotPerSec = leftVelocityRotPerSec.getValue();
    inputs.leftCurrentAmps = leftCurrentAmps.getValue();

    inputs.rightVelocityRotPerSec = leftVelocityRotPerSec.getValue();
    inputs.rightCurrentAmps = leftCurrentAmps.getValue();

    inputs.rotatorPositionRot = rotatorPositionRot.getValue();
    inputs.rotatorClosedLoopErrorRot = rotatorClosedLoopErrorRot.getValueAsDouble();
    inputs.rotatorVelocityRotPerSec = rotatorVelocityRotPerSec.getValue();
  }

    public void addInstruments(Orchestra orchestra) {
      orchestra.addInstrument(rotator);
      orchestra.addInstrument(intakeLeft);
      orchestra.addInstrument(intakeRight);
  }

  public void setRotationAngle(Angle angle) {
    rotator.setControl(new PositionVoltage(angle.in(Rotations)));
  }

  public void setIntakeVelocity(AngularVelocity velocity) {
    intakeLeft.setControl(intakeSpin.withVelocity(velocity.in(RotationsPerSecond)));
    intakeRight.setControl(intakeSpin.withVelocity(-velocity.in(RotationsPerSecond)));
  }

  public void setStaticBrake() {
    rotator.setControl(new StaticBrake());
  }

}