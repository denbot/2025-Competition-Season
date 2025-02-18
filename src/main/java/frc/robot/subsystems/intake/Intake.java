// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakerLeft =
      new TalonFX(IntakeConstants.INTAKER_LEFT_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX intakerRight =
      new TalonFX(IntakeConstants.INTAKER_RIGHT_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX rotationLeft =
      new TalonFX(IntakeConstants.ROTATION_RIGHT_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotationLeftEncoder =
      new CANcoder(IntakeConstants.ROTATION_LEFT_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final TalonFX rotationRight =
      new TalonFX(IntakeConstants.ROTATION_RIGHT_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotationRightEncoder =
      new CANcoder(IntakeConstants.ROTATION_RIGHT_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final TalonFX indexerLeft =
      new TalonFX(IntakeConstants.INDEXER_LEFT_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX indexerRight =
      new TalonFX(IntakeConstants.INDEXER_RIGHT_MOTOR_ID, OperatorConstants.canivoreSerial);

  public static final TalonFXConfiguration leftRotationConfig =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  // .withFeedbackRemoteSensorID(IntakeConstants.EXTENDER_MOTOR_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(IntakeConstants.rotatorGearRatio))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withForwardSoftLimitThreshold(IntakeConstants.forwardSoftLimit)
                  .withReverseSoftLimitEnable(false)
                  .withReverseSoftLimitThreshold(IntakeConstants.reverseSoftLimit))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(4)
                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(
              new Slot0Configs()
                  .withKP(1)
                  .withKD(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine));

  public static final TalonFXConfiguration rightRotationConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withFeedback(
              new FeedbackConfigs()
                  // .withFeedbackRemoteSensorID(IntakeConstants.EXTENDER_MOTOR_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(IntakeConstants.rotatorGearRatio))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withForwardSoftLimitThreshold(IntakeConstants.forwardSoftLimit)
                  .withReverseSoftLimitEnable(false)
                  .withReverseSoftLimitThreshold(IntakeConstants.reverseSoftLimit))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(4)
                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(
              new Slot0Configs()
                  .withKP(1)
                  .withKD(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine));

  public Intake() {
    rotationLeft.setNeutralMode(NeutralModeValue.Brake);
    rotationRight.setNeutralMode(NeutralModeValue.Brake);

    intakerLeft.setNeutralMode(NeutralModeValue.Coast);
    intakerRight.setNeutralMode(NeutralModeValue.Coast);
    indexerLeft.setNeutralMode(NeutralModeValue.Coast);
    indexerRight.setNeutralMode(NeutralModeValue.Coast);

    rotationLeft.getConfigurator().apply(leftRotationConfig);
    rotationRight.getConfigurator().apply(rightRotationConfig);
  }

  public double getLeftRotationAngle() {
    return rotationLeftEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public double getRightRotationAngle() {
    return rotationRightEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public void setLeftAngle(double angle) {
    rotationLeft.setControl(new PositionVoltage(angle / 360));
  }

  public void setRightAngle(double angle) {
    rotationRight.setControl(new PositionVoltage(angle / 360));
  }

  public void setLeftIntakerSpeed(double speed) {
    intakerLeft.setControl(new VoltageOut(speed));
  }

  public void setRightIntakerSpeed(double speed) {
    intakerRight.setControl(new VoltageOut(speed));
  }

  public void setLeftIndexerSpeed(double speed) {
    indexerLeft.setControl(new VoltageOut(speed));
    ;
  }

  public void setRightIndexerSpeed(double speed) {
    indexerRight.setControl(new VoltageOut(speed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Rotation Angle", getLeftRotationAngle());
    SmartDashboard.putNumber("Right Rotation Angle", getRightRotationAngle());
    SmartDashboard.putNumber(
        "Left Intaker Velocity", intakerLeft.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Right Intaker Velocity", intakerRight.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Left Indexer Velocity", indexerLeft.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Right Indexer Velocity", indexerRight.getRotorVelocity().getValueAsDouble());
  }
}
