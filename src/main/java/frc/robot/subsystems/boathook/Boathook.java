// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.OperatorConstants;

public class Boathook extends SubsystemBase {
  /** Creates a new Boathook. */
  private static final TalonFX rotationMotor =
      new TalonFX(BoathookConstants.ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX extenderMotor =
      new TalonFX(BoathookConstants.EXTENDER_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final CANcoder rotationEncoder =
      new CANcoder(BoathookConstants.ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANcoder extentionEncoder =
      new CANcoder(BoathookConstants.EXTENDER_ENCODER_ID, OperatorConstants.canivoreSerial);

  public double angle1;
  public double length1;

  public double angle2;
  public double length2;

  public double angle3;
  public double length3;

  public static final TalonFXConfiguration rotationConfig =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  // .withFeedbackRemoteSensorID(BoathookConstants.EXTENDER_MOTOR_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(BoathookConstants.rotatorGearRatio))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withForwardSoftLimitThreshold(BoathookConstants.fowardSoftLimit)
                  .withReverseSoftLimitEnable(false)
                  .withReverseSoftLimitThreshold(BoathookConstants.reverseSoftLimit))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(4)
                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(
              new Slot0Configs()
                  .withKP(15)
                  .withKD(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withHardwareLimitSwitch(
              new HardwareLimitSwitchConfigs()
                  .withForwardLimitEnable(true)
                  .withForwardLimitAutosetPositionEnable(true)
                  .withForwardLimitAutosetPositionValue(0)
                  .withForwardLimitRemoteSensorID(BoathookConstants.ROTATOR_CANDI_ID)
                  .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANdiS1)
                  .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed));

  public static final TalonFXConfiguration extenderConfig =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(BoathookConstants.EXTENDER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withSensorToMechanismRatio(BoathookConstants.rotatorGearRatio))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withForwardSoftLimitThreshold(BoathookConstants.EXTENDER_FOWARD_LIMIT)
                  .withReverseSoftLimitEnable(false)
                  .withReverseSoftLimitThreshold(BoathookConstants.EXTENDER_REVERSE_LIMIT))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(2)
                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(new Slot0Configs().withKP(.1).withKD(0).withKG(0))
          .withHardwareLimitSwitch(
              new HardwareLimitSwitchConfigs()
                  .withForwardLimitEnable(true)
                  .withForwardLimitAutosetPositionEnable(true)
                  .withForwardLimitAutosetPositionValue(0)
                  .withForwardLimitRemoteSensorID(BoathookConstants.EXTENDER_CANDI_ID)
                  .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANdiS1)
                  .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed));

  public Boathook() {
    rotationMotor.setNeutralMode(NeutralModeValue.Brake);

    extenderMotor.setNeutralMode(NeutralModeValue.Brake);

    rotationMotor.getConfigurator().apply(rotationConfig);
  }

  public double getRotationAngle() {
    return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public void setAngle(double angle) {
    rotationMotor.setControl(new PositionVoltage(angle / 360));
  }

  public double getAngle() {
    StatusSignal<Angle> angle = rotationMotor.getPosition();
    return angle.getValueAsDouble() * 360;
  }

  public void setLength(double length) {
    rotationMotor.setControl(new PositionVoltage(length));
  }

  public double getLength() {
    return extentionEncoder.getPosition().getValueAsDouble();
    // might have to multiple by something
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
