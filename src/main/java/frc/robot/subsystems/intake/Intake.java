// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
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

  private final CANdi intakeSensors =
      new CANdi(IntakeConstants.CANDI_ID, OperatorConstants.canivoreSerial);

  public static final TalonFXConfiguration intakeRotationConfig =
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
                  .withKP(63.5)
                  .withKD(12)
                  .withKG(2)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withSlot1(
              new Slot1Configs()
                  .withKP(20)
                  .withKD(13)
                  .withKG(2)
                  .withGravityType(GravityTypeValue.Arm_Cosine));

  public static final CANcoderConfiguration intakeRotationSensorConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(0.085)
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

  public boolean up = false;
  public boolean stop = true;

  private static final CANdiConfiguration intakeSensorsConfig =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs()
                  .withS1CloseState(S1CloseStateValue.CloseWhenNotFloating)
                  .withS1FloatState(S1FloatStateValue.FloatDetect)
                  .withS2CloseState(S2CloseStateValue.CloseWhenNotFloating)
                  .withS2FloatState(S2FloatStateValue.FloatDetect));

  private static final TalonFXConfiguration intakeConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(30))
          .withSlot0(new Slot0Configs().withKS(5.4).withKP(3));

  private static final NeutralOut motorStop = new NeutralOut();
  private static final VelocityTorqueCurrentFOC intakeSpin =
      new VelocityTorqueCurrentFOC(0).withAcceleration(IntakeConstants.intakeAcceleration);

  private static final PositionTorqueCurrentFOC intakeMove = new PositionTorqueCurrentFOC(0);

  public Intake() {
    rotation.setNeutralMode(NeutralModeValue.Brake);
    intakeLeft.setNeutralMode(NeutralModeValue.Coast);
    intakeRight.setNeutralMode(NeutralModeValue.Coast);

    intakeLeft.getConfigurator().apply(intakeConfig);
    intakeRight.getConfigurator().apply(intakeConfig);

    rotation.getConfigurator().apply(intakeRotationConfig);
    rotationEncoder.getConfigurator().apply(intakeRotationSensorConfig);

    NamedCommands.registerCommand(
        "IntakeDown", new IntakeMoveCommand(this, false, IntakeConstants.intakeDownAngle, 1, -3));
    intakeSensors.getConfigurator().apply(intakeSensorsConfig);
  }

  public double getRotationAngle() {
    return rotation.getRotorPosition().getValueAsDouble() * 360.0;
  }

  public void setAngle(double angle, int slot, double feedForward) {
    rotation.setControl(intakeMove.withPosition(angle).withSlot(slot).withFeedForward(feedForward));
  }

  public void setIntakeSpeed(double velocity) {
    intakeLeft.setControl(intakeSpin.withVelocity(velocity));
    intakeRight.setControl(intakeSpin.withVelocity(-velocity));
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

  public void stopIntake() {
    intakeLeft.setControl(motorStop);
    intakeRight.setControl(motorStop);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Rotation Angle", getRotationAngle());
  }

  public boolean isCoralIntaken() {
    // If either switch is triggered. This will eventually be one switch
    return intakeSensors.getS1Closed().getValue() || intakeSensors.getS2Closed().getValue();
  }
}
