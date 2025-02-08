// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.OperatorConstants;

public class Boathook extends SubsystemBase {
  /** Creates a new Boathook. */
  private final TalonFX rotationMotor =
      new TalonFX(BoathookConstants.ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX extenderMotor =
      new TalonFX(BoathookConstants.EXTENDER_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final CANcoder rotationEncoder =
      new CANcoder(BoathookConstants.EXTENDER_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final CANdi proximitySensor = new CANdi(BoathookConstants.CANDI_ID);

  public static final TalonFXConfiguration rotationConfig =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(BoathookConstants.EXTENDER_MOTOR_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(BoathookConstants.rotatorGearRatio))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(BoathookConstants.fowardSoftLimit)
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(BoathookConstants.reverseSoftLimit))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(2)
                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(
              new Slot0Configs()
                  .withKP(1)
                  .withKD(1)
                  .withKG(1)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withHardwareLimitSwitch(
              new HardwareLimitSwitchConfigs()
                  .withForwardLimitEnable(true)
                  .withForwardLimitAutosetPositionEnable(true)
                  .withForwardLimitAutosetPositionValue(0)
                  .withForwardLimitRemoteSensorID(BoathookConstants.CANDI_ID)
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
