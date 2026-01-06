// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor intakeLeftMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor intakeRightMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor rotatorMotor = DCMotor.getKrakenX60Foc(1);

  private DCMotorSim intakeLeftSim;
  private DCMotorSim intakeRightSim;
  private DCMotorSim intakeRotatorSim;

  private final PIDController rotatorController = new PIDController(0.1, 0, 0);
  private final PIDController intakeController = new PIDController(1.0, 0, 0);

  private double intakeAppliedVolts = 0.0;
  private double rotatorAppliedVolts = 0.0;

  public IntakeIOSim(){
    intakeLeftSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.008, 1),
    intakeLeftMotor);

    intakeRightSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.008, 1),
    intakeRightMotor);

    intakeRotatorSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.1, 120),
    rotatorMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeAppliedVolts = intakeController.calculate(intakeLeftSim.getAngularVelocity().in(RotationsPerSecond));
    rotatorAppliedVolts = rotatorController.calculate(intakeRotatorSim.getAngularPosition().in(Degrees));

    intakeLeftSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));
    intakeLeftSim.update(0.02);

    intakeRightSim.setInputVoltage(MathUtil.clamp(-intakeAppliedVolts, -12.0, 12.0));
    intakeRightSim.update(0.02);

    intakeRotatorSim.setInputVoltage(MathUtil.clamp(rotatorAppliedVolts, -12.0, 12.0));
    intakeRotatorSim.update(0.02);

    inputs.intakeLeftConnected = true;
    inputs.intakeRightConnected = true;
    inputs.rotatorConnected = true;
    inputs.coralSensorConnected = true;

    //Sim Velocity defaults to Rad/sec. 60 rps = 377 Rad/sec
    inputs.leftVelocityRotPerSec = RotationsPerSecond.of(intakeLeftSim.getAngularVelocityRPM() / 60.0);
    inputs.leftCurrentAmps = Amp.of(intakeLeftSim.getCurrentDrawAmps());

    inputs.rightVelocityRotPerSec = RotationsPerSecond.of(intakeRightSim.getAngularVelocityRPM() / 60.0);
    inputs.rightCurrentAmps = Amp.of(intakeRightSim.getCurrentDrawAmps());

    inputs.rotatorPositionRot = intakeRotatorSim.getAngularPosition();
    inputs.rotatorVelocityRotPerSec = RotationsPerSecond.of(intakeRotatorSim.getAngularVelocityRPM() / 60.0);
    inputs.rotatorClosedLoopErrorRot = rotatorController.getError();

    inputs.stateS1 = S1StateValue.High;
    inputs.stateS2 = S2StateValue.High;
  }

  public void setRotationAngle(Angle angle) {
    rotatorController.setSetpoint(angle.in(Degrees));
  }

  public void setIntakeVelocity(AngularVelocity velocity) {
    intakeController.setSetpoint(velocity.in(RotationsPerSecond));
  }

  public void setStaticBrake() {}
}