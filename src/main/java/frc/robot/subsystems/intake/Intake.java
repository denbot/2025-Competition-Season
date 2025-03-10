// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import frc.robot.Robot;
import frc.robot.commands.intakeCommands.IntakeMoveCommand;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeLeft =
      new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX intakeRight =
      new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX rotation =
      new TalonFX(IntakeConstants.INTAKE_ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);
  private final CANcoder rotationEncoder =
      new CANcoder(IntakeConstants.INTAKE_ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  public static final TalonFXConfiguration intakeRotationConfig =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
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

  public boolean up = false;
  public boolean stop = true;

  public Intake() {
    rotation.setNeutralMode(NeutralModeValue.Brake);
    intakeLeft.setNeutralMode(NeutralModeValue.Coast);
    intakeRight.setNeutralMode(NeutralModeValue.Coast);

    rotation.getConfigurator().apply(intakeRotationConfig);
    rotationEncoder.getConfigurator().apply(intakeRotationSensorConfig);

    NamedCommands.registerCommand("IntakeDown", new IntakeMoveCommand(this, false));
  }

  public double getRotationAngle() {
    return rotation.getRotorPosition().getValueAsDouble() * 360.0;
  }

  public void setAngle(double angle) {
    rotation.setControl(new PositionVoltage(angle));
  }

  public void setIntakeSpeed(double speed) {
    intakeLeft.setControl(new VoltageOut(speed));
    intakeRight.setControl(new VoltageOut(-speed));
  }

  public void flipStop() {
    stop = !stop;
  }

  public void flipUp() {
    up = !up;
  }

  public void addInstruments() {
    // Add a single device to the orchestra
    Robot.robotContainer.m_orchestra.addInstrument(rotation);
    Robot.robotContainer.m_orchestra.addInstrument(intakeLeft);
    Robot.robotContainer.m_orchestra.addInstrument(intakeRight);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Rotation Angle", getRotationAngle());
  }
}
