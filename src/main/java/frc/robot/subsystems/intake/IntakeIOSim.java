// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeLeftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.001, 1),
          DCMotor.getCIM(1));

    private DCMotorSim intakeRightSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.001, 1),
    DCMotor.getCIM(1));

    private DCMotorSim intakeRotatorSim =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.001, 1),
    DCMotor.getCIM(1));

  private double intakeAppliedVolts = 0.0;
  private double rotationAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeLeftSim.setInputVoltage(intakeAppliedVolts);
    intakeLeftSim.update(0.02);

    intakeRightSim.setInputVoltage(-intakeAppliedVolts);
    intakeRightSim.update(0.02);

    intakeRotatorSim.setInputVoltage(rotationAppliedVolts);
    intakeRotatorSim.update(0.02);

    inputs.leftVelocityRevPerSec = intakeLeftSim.getAngularVelocityRPM() / 60.0;
    inputs.leftCurrentAmps = intakeLeftSim.getCurrentDrawAmps();

    inputs.rightVelocityRevPerSec = intakeRightSim.getAngularVelocityRPM() / 60.0;
    inputs.rightCurrentAmps = intakeRightSim.getCurrentDrawAmps();

    inputs.rotatorPositionDeg = Units.rotationsToDegrees(intakeRotatorSim.getAngularPositionRotations());
    inputs.rotatorVelocityRevPerSec = intakeRotatorSim.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void setIntakeSpeed(double volts) {
    intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

//   public void setIntakeHoldingVoltage(double voltage) {
//     intakeLeftSim.setControl(new VoltageOut(voltage));
//     intakeRightSim.setControl(new VoltageOut(-voltage));
//   }

//   public double getrotationAngle() {
//     return rotationSim.getPosition().getValueAsDouble();
//   }

//   public boolean getIntakeIsStalled() {
//     boolean returnCondition =
//         intakeRightSim.getStatorCurrent().getValueAsDouble() > 40
//             && intakeRightSim.getVelocity().getValueAsDouble() < 1;
//     if (returnCondition) System.out.println("Motor Is Stalled!!!");
//     return returnCondition;
//   }

//   public double getClosedLoopError() {
//     return rotationSim.getClosedLoopError().getValueAsDouble();
//   }

//   public double getrotationSimSetpoint() {
//     double rotationSimReference = rotationSim.getClosedLoopReference().getValueAsDouble();
//     return rotationSimReference;
//   }

//   public double getrotationSimVelocity() {
//     return rotationSim.getVelocity().getValueAsDouble();
//   }

//   public void setAngle(double angle) {
//     rotationSim.setControl(new PositionVoltage(angle));
//   }

//   public void setIntakeSpeed(double velocity) {
//     intakeLeftSim.setControl(intakeSpin.withVelocity(velocity));
//     intakeRightSim.setControl(intakeSpin.withVelocity(-velocity));
//   }

//   public void setStaticBrake() {
//     rotationSim.setControl(new StaticBrake());
//   }

//   public void stopIntake() {
//     intakeLeftSim.setControl(motorStop);
//     intakeRightSim.setControl(motorStop);
//   }
}