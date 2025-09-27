// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boathook;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;

public class Boathook extends SubsystemBase {
  /** Creates a new Boathook. */

  // ONLY HERE UNTIL INTAKE IS FINISHED, currently only refferenced in runIntakeCommand.java
  public enum Level {
    L1(
        BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION,
        BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION,
        BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION),
    L2(
        BoathookConstants.IDLE_ANGLE, BoathookConstants.L2_EXTENSION,
        BoathookConstants.L2_SETUP_ANGLE, BoathookConstants.L2_EXTENSION,
        BoathookConstants.L2_SCORE_ANGLE, BoathookConstants.IDLE_EXTENSION),
    L3(
        BoathookConstants.IDLE_ANGLE, BoathookConstants.L3_EXTENSION,
        BoathookConstants.L3_SETUP_ANGLE, BoathookConstants.L3_EXTENSION,
        BoathookConstants.L3_SCORE_ANGLE, BoathookConstants.L2_EXTENSION),
    L4(
        BoathookConstants.L4_SETUP_ANGLE - 2,
        BoathookConstants.L4_EXTENSION,
        BoathookConstants.L4_SETUP_ANGLE,
        BoathookConstants.L3_EXTENSION,
        BoathookConstants.IDLE_ANGLE,
        BoathookConstants.IDLE_EXTENSION);

    public double angle1;
    public double length1;
    public double angle2;
    public double length2;
    public double angle3;
    public double length3;

    Level(
        double angle1,
        double length1,
        double angle2,
        double length2,
        double angle3,
        double length3) {
      this.angle1 = angle1;
      this.length1 = length1;
      this.angle2 = angle2;
      this.length2 = length2;
      this.angle3 = angle3;
      this.length3 = length3;
    }
  }

  private Level level = Level.L1;
  public double microRotationOffset = 0.0;

  private static final TalonFX rotationMotor =
      new TalonFX(BoathookConstants.ROTATION_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final TalonFX extenderMotor =
      new TalonFX(BoathookConstants.EXTENDER_MOTOR_ID, OperatorConstants.canivoreSerial);

  private final CANcoder rotationEncoder =
      new CANcoder(BoathookConstants.ROTATION_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANcoder extensionEncoder =
      new CANcoder(BoathookConstants.EXTENDER_ENCODER_ID, OperatorConstants.canivoreSerial);

  private final CANdi limitSensors =
      new CANdi(BoathookConstants.CANDI_ID, OperatorConstants.canivoreSerial);

  public static final TalonFXConfiguration rotationConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(BoathookConstants.ROTATION_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(BoathookConstants.ROTATOR_GEAR_RATIO))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(BoathookConstants.ROTATOR_FORWARD_LIMIT))
          // .withMotionMagic(
          //     new MotionMagicConfigs()
          //         .withMotionMagicAcceleration(4)
          //         .withMotionMagicCruiseVelocity(1))
          .withSlot0(
              new Slot0Configs()
                  .withKP(20)
                  .withKD(0)
                  .withKS(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
          .withHardwareLimitSwitch(
              new HardwareLimitSwitchConfigs()
                  .withReverseLimitEnable(true)
                  .withReverseLimitAutosetPositionEnable(true)
                  .withReverseLimitAutosetPositionValue(BoathookConstants.ROTATOR_REVERSE_LIMIT)
                  .withReverseLimitRemoteSensorID(BoathookConstants.CANDI_ID)
                  .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS1)
                  .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                  .withForwardLimitEnable(false)
                  .withForwardLimitAutosetPositionEnable(false));

  private static final CANcoderConfiguration rotationEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(-0.44241328125)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

  public static final TalonFXConfiguration extenderConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackRemoteSensorID(BoathookConstants.EXTENDER_ENCODER_ID)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withRotorToSensorRatio(25 * (34.0 / 24.0))
                  .withSensorToMechanismRatio(1.8)
                  .withFeedbackRotorOffset(0))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(BoathookConstants.EXTENDER_FORWARD_LIMIT)
                  .withReverseSoftLimitEnable(false))
          //          .withMotionMagic(
          //              new MotionMagicConfigs()
          //                  .withMotionMagicAcceleration(2)
          //                  .withMotionMagicCruiseVelocity(1))
          .withSlot0(new Slot0Configs().withKP(20).withKD(0).withKG(0))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withHardwareLimitSwitch(
              new HardwareLimitSwitchConfigs()
                  .withReverseLimitEnable(true)
                  .withReverseLimitAutosetPositionEnable(true)
                  .withReverseLimitAutosetPositionValue(BoathookConstants.EXTENDER_REVERSE_LIMIT)
                  .withReverseLimitRemoteSensorID(BoathookConstants.CANDI_ID)
                  .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS2)
                  .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                  .withForwardLimitEnable(false)
                  .withForwardLimitAutosetPositionEnable(false));

  CANcoderConfiguration extensionEncoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(0.19402421875) // 0.372
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

  CANdiConfiguration limitSensorsConfig =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs()
                  .withS1CloseState(S1CloseStateValue.CloseWhenLow)
                  .withS1FloatState(S1FloatStateValue.FloatDetect)
                  .withS2CloseState(S2CloseStateValue.CloseWhenLow)
                  .withS2FloatState(S2FloatStateValue.FloatDetect));

  public Boathook() {
    rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    extenderMotor.setNeutralMode(NeutralModeValue.Brake);
    rotationMotor.getConfigurator().apply(rotationConfig);
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    extenderMotor.getConfigurator().apply(extenderConfig);
    extensionEncoder.getConfigurator().apply(extensionEncoderConfig);
    limitSensors.getConfigurator().apply(limitSensorsConfig);
  }

  public void setAngle(double angle) {
    rotationMotor.setControl(new PositionVoltage(angle / 360.0));
  }

  public double getAngle() {
    StatusSignal<Angle> angle = rotationMotor.getPosition();
    return angle.getValueAsDouble() * 360.0;
  }

  public double getAngleSetpoint() {
    StatusSignal<Double> angle = rotationMotor.getClosedLoopReference();
    return angle.getValueAsDouble() * 360;
  }

  public void setLength(double length) {
    extenderMotor.setControl(new PositionVoltage(length));
  }

  public double getLength() {
    StatusSignal<Angle> length = extenderMotor.getPosition();
    return length.getValueAsDouble();
  }

  public double getLengthSetpoint() {
    return extenderMotor.getClosedLoopReference().getValueAsDouble();
  }

  public void setBrakeExtender() {
    extenderMotor.setControl(new StaticBrake());
  }

  public void setLevel(Level incomingLevel) {
    this.level = incomingLevel;
  }

  public Level getLevel() {
    return level;
  }

  public void addInstruments() {
    Robot.robotContainer.m_orchestra.addInstrument(rotationMotor);
    Robot.robotContainer.m_orchestra.addInstrument(extenderMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Boathook Angle", getAngle());
    SmartDashboard.putNumber("Boathook Extension", getLength());
  }
}
