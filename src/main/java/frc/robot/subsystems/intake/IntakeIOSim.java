// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.Orchestra;

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

  private final PIDController rotatorController = new PIDController(0.5, 0, 0);
  private final PIDController intakeController = new PIDController(10, 0, 0);

  private double intakeAppliedVolts = 0.0;
  private double rotatorAppliedVolts = 0.0;

  public IntakeIOSim(){
    intakeLeftSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
    intakeLeftMotor);

    intakeRightSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
    intakeRightMotor);

    intakeRotatorSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
    rotatorMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeAppliedVolts = intakeController.calculate(intakeLeftSim.getAngularPositionRad());
    rotatorAppliedVolts = rotatorController.calculate(intakeRotatorSim.getAngularVelocityRPM());

    intakeLeftSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));
    intakeLeftSim.update(0.02);

    intakeRightSim.setInputVoltage(MathUtil.clamp(-intakeAppliedVolts, -12.0, 12.0));
    intakeRightSim.update(0.02);

    intakeRotatorSim.setInputVoltage(MathUtil.clamp(rotatorAppliedVolts, -12.0, 12.0));
    intakeRotatorSim.update(0.02);

    inputs.leftVelocityRevPerSec = RevolutionsPerSecond.of(intakeLeftSim.getAngularVelocityRPM() / 60.0);
    inputs.leftCurrentAmps = Amp.of(intakeLeftSim.getCurrentDrawAmps());

    inputs.rightVelocityRevPerSec = RevolutionsPerSecond.of(intakeRightSim.getAngularVelocityRPM() / 60.0);
    inputs.rightCurrentAmps = Amp.of(intakeRightSim.getCurrentDrawAmps());

    inputs.rotatorPositionDeg = intakeRotatorSim.getAngularPosition();
    inputs.rotatorVelocityRevPerSec = RevolutionsPerSecond.of(intakeRotatorSim.getAngularVelocityRPM() / 60.0);
  }

  @Override
  public void addInstruments(Orchestra orchestra){
    System.out.println("Unable to add instruments in simulation.");
  }

  public void setAngle(Angle angle) {
    rotatorController.setSetpoint(angle.in(Radians));
  }

  public void setIntakeSpeed(AngularVelocity velocity) {
    intakeController.setSetpoint(velocity.in(RevolutionsPerSecond));
  }

  public void setStaticBrake() {}
}