// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intakeCommands.FunnelIntake;
import frc.robot.commands.intakeCommands.StopIntake;

public class Intake extends SubsystemBase {
  private final TalonFX rotation =
      new TalonFX(IntakeConstants.INTAKE_ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotationEncoder =
      new CANcoder(IntakeConstants.INTAKE_ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final TalonFX indexerLeft =
      new TalonFX(IntakeConstants.INDEXER_LEFT_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX indexerRight =
      new TalonFX(IntakeConstants.INDEXER_RIGHT_MOTOR_ID, OperatorConstants.canivoreSerial);

  public static final TalonFXConfiguration intakeRotationConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(IntakeConstants.INTAKE_ROTATION_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
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
                  .withKP(10)
                  .withKD(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine));

  public static final CANcoderConfiguration intakeRotationSensorConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(0.001)
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

  public Intake() {
    rotation.setNeutralMode(NeutralModeValue.Brake);

    indexerLeft.setNeutralMode(NeutralModeValue.Coast);
    indexerRight.setNeutralMode(NeutralModeValue.Coast);

    rotation.getConfigurator().apply(intakeRotationConfig);
    rotationEncoder.getConfigurator().apply(intakeRotationSensorConfig);

    NamedCommands.registerCommand("IntakeDown", new StopIntake(this));
    NamedCommands.registerCommand("Funnel", new FunnelIntake(this));
  }

  // public double getRotationAngle() {
  //   return rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  // }

  public void setAngle(double angle) {
    rotation.setControl(new PositionVoltage(angle / 360));
  }

  public void setLeftIndexerSpeed(double speed) {
    indexerLeft.setControl(new VoltageOut(speed));
  }

  public void setRightIndexerSpeed(double speed) {
    indexerRight.setControl(new VoltageOut(speed));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Intake Rotation Angle", getRotationAngle());
    SmartDashboard.putNumber(
        "Left Indexer Velocity", indexerLeft.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Right Indexer Velocity", indexerRight.getRotorVelocity().getValueAsDouble());
  }
}
