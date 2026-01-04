
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

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
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.OperatorConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeLeft =
      new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX intakeRight =
      new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX rotation =
      new TalonFX(IntakeConstants.INTAKE_ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotationEncoder =
      new CANcoder(IntakeConstants.INTAKE_ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANdi intakeSensors =
      new CANdi(IntakeConstants.CANDI_ID, OperatorConstants.canivoreSerial);

  private static final NeutralOut motorStop = new NeutralOut();
  private static final VelocityTorqueCurrentFOC intakeSpin =
      new VelocityTorqueCurrentFOC(0).withAcceleration(IntakeConstants.intakeAcceleration);

  private final StatusSignal<AngularVelocity> leftVelocityRotPerSec = intakeLeft.getVelocity();
  private final StatusSignal<Current> leftCurrentAmps = intakeLeft.getSupplyCurrent();

  private final StatusSignal<AngularVelocity> rightVelocityRotPerSec = intakeRight.getVelocity();
  private final StatusSignal<Current> rightCurrentAmps = intakeRight.getSupplyCurrent();

  private final StatusSignal<Angle> rotatorPositionDeg = intakeRight.getPosition();
  private final StatusSignal<AngularVelocity> rotatorVelocityRotPerSec = intakeRight.getVelocity();

  public IntakeIOTalonFX() {
    var intakeRotationConfig =
    new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(70))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(IntakeConstants.INTAKE_ROTATION_ENCODER_ID)
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

    var intakeRotationSensorConfig =
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

    rotation.setNeutralMode(NeutralModeValue.Brake);
    intakeLeft.setNeutralMode(NeutralModeValue.Coast);
    intakeRight.setNeutralMode(NeutralModeValue.Coast);

    tryUntilOk(5, () -> rotation.getConfigurator().apply(intakeRotationConfig, 0.25));
    tryUntilOk(5, () -> rotationEncoder.getConfigurator().apply(intakeRotationSensorConfig, 0.25));
    tryUntilOk(5, () -> intakeLeft.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> intakeRight.getConfigurator().apply(intakeConfig, 0.25));
    tryUntilOk(5, () -> intakeSensors.getConfigurator().apply(intakeSensorsConfig,0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leftVelocityRotPerSec, leftCurrentAmps,
        rightVelocityRotPerSec, rightCurrentAmps,
        rotatorPositionDeg, rotatorVelocityRotPerSec);

    intakeRight.optimizeBusUtilization();
    intakeLeft.optimizeBusUtilization();
    rotation.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftVelocityRotPerSec, leftCurrentAmps,
    rightVelocityRotPerSec, rightCurrentAmps,
    rotatorPositionDeg, rotatorVelocityRotPerSec);

    inputs.leftVelocityRevPerSec = leftVelocityRotPerSec.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();

    inputs.rightVelocityRevPerSec = leftVelocityRotPerSec.getValueAsDouble();
    inputs.rightCurrentAmps = leftCurrentAmps.getValueAsDouble();

    inputs.rotatorPositionDeg = Units.rotationsToDegrees(rotatorPositionDeg.getValueAsDouble());
    inputs.rotatorVelocityRevPerSec = rotatorVelocityRotPerSec.getValueAsDouble();
  }

    public void addInstruments(Orchestra orchestra) {
      orchestra.addInstrument(rotation);
      orchestra.addInstrument(intakeLeft);
      orchestra.addInstrument(intakeRight);
  }

  public void setIntakeHoldingVoltage(double voltage) {
    intakeLeft.setControl(new VoltageOut(voltage));
    intakeRight.setControl(new VoltageOut(-voltage));
  }

  public double getClosedLoopError() {
    return rotation.getClosedLoopError().getValueAsDouble();
  }

  public double getRotationSetpoint() {
    double rotationReference = rotation.getClosedLoopReference().getValueAsDouble();
    return rotationReference;
  }

  public double getRotationVelocity() {
    return rotation.getVelocity().getValueAsDouble();
  }

  public void setAngle(double angle) {
    rotation.setControl(new PositionVoltage(angle));
  }

  public void setIntakeSpeed(double velocity) {
    intakeLeft.setControl(intakeSpin.withVelocity(velocity));
    intakeRight.setControl(intakeSpin.withVelocity(-velocity));
  }

  public void setStaticBrake() {
    rotation.setControl(new StaticBrake());
  }

  public void stopIntake() {
    intakeLeft.setControl(motorStop);
    intakeRight.setControl(motorStop);
  }

}