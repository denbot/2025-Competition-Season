// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.boathookCommands.BoathookExtendMotionPathCommand;
import frc.robot.commands.boathookCommands.BoathookRetractMotionPathCommand;

public class Boathook extends SubsystemBase {
  /** Creates a new Boathook. */
  private static final TalonFX rotationMotor =
      new TalonFX(BoathookConstants.ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX extenderMotor =
      new TalonFX(BoathookConstants.EXTENDER_MOTOR_ID, OperatorConstants.canivoreSerial);

  //   private final CANcoder rotationEncoder =
  //       new CANcoder(BoathookConstants.ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANcoder extensionCancoder =
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
                  .withSensorToMechanismRatio(BoathookConstants.ROTATOR_GEAR_RATIO))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(BoathookConstants.ROTATOR_FORWARD_LIMIT)
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(BoathookConstants.ROTATOR_REVERSE_LIMIT))
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
                  .withForwardLimitRemoteSensorID(BoathookConstants.BOATHOOK_CANDI_ID)
                  .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANdiS1)
                  .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed));

  public static final TalonFXConfiguration extenderConfig =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(BoathookConstants.EXTENDER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(25 * (34.0 / 24.0))
                  .withSensorToMechanismRatio(1.8))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withForwardSoftLimitThreshold(BoathookConstants.EXTENDER_FORWARD_LIMIT)
                  .withReverseSoftLimitEnable(false)
                  .withReverseSoftLimitThreshold(BoathookConstants.EXTENDER_REVERSE_LIMIT))
//          .withMotionMagic(
//              new MotionMagicConfigs()
//                  .withMotionMagicAcceleration(2)
//                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(new Slot0Configs().withKP(28).withKD(0).withKG(0))
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive)
            )
              .withSoftwareLimitSwitch(
                      new SoftwareLimitSwitchConfigs()
                              .withReverseSoftLimitEnable(true)
                              .withReverseSoftLimitThreshold(0)
                              .withForwardSoftLimitEnable(true)
                              .withForwardSoftLimitThreshold(4.5)
              )
//          .withHardwareLimitSwitch(
//              new HardwareLimitSwitchConfigs()
//                  .withReverseLimitEnable(true)
//                  .withReverseLimitAutosetPositionEnable(true)
//                  .withReverseLimitAutosetPositionValue(4)
//                  .withReverseLimitRemoteSensorID(BoathookConstants.BOATHOOK_CANDI_ID)
//                  .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS2)
//                  .withReverseLimitType(ReverseLimitTypeValue.NormallyClosed));

  CANcoderConfiguration extentionCANcoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

  public Boathook() {
    rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    extenderMotor.setNeutralMode(NeutralModeValue.Brake);
    rotationMotor.getConfigurator().apply(rotationConfig);
    extenderMotor.getConfigurator().apply(extenderConfig);
    extensionCancoder.getConfigurator().apply(extentionCANcoderConfig);

    NamedCommands.registerCommand("BoathookExtend", new BoathookExtendMotionPathCommand(this));
    NamedCommands.registerCommand("BoathookRetract", new BoathookRetractMotionPathCommand(this));
  }

  //   public double getRotationAngle() {
  //     return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  //   }

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
    return extensionCancoder.getPosition().getValueAsDouble();
    // might have to multiple by something
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
