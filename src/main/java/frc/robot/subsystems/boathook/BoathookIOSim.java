package frc.robot.subsystems.boathook;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class BoathookIOSim implements BoathookIO {
  private static final DCMotor angleMotor = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor extensionMotor = DCMotor.getKrakenX60Foc(1);

  // I'm pretty sure these values are just whatever makes the sim be responsive in the way you want
  private final PIDController angleController = new PIDController(0.5, 0, 0);
  private final PIDController extensionController = new PIDController(10, 0, 0);

  private final DCMotorSim angleSim;
  private final DCMotorSim extensionSim;

  private boolean angleClosedLoop = false;
  private boolean extensionClosedLoop = true;

  private double angleAppliedVolts = 0.0;
  private double extensionAppliedVolts = 0.0;

  public BoathookIOSim() {
    angleSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(angleMotor, 0.001, 1),
        angleMotor
    );

    extensionSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(extensionMotor, 0.001, 1),
        extensionMotor
    );
  }

  @Override
  public void updateInputs(BoathookIOInputs inputs) {
    // Run closed-loop control
    if(angleClosedLoop) {
      angleAppliedVolts = angleController.calculate(angleSim.getAngularPositionRad());
    } else {
      angleController.reset();
    }

    if(extensionClosedLoop) {
      extensionAppliedVolts = extensionController.calculate(extensionSim.getAngularPositionRad());
    } else {
      extensionController.reset();
    }

    // Update simulation state
    angleSim.setInputVoltage(MathUtil.clamp(angleAppliedVolts, -12.0, 12.0));
    extensionSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));
    angleSim.update(0.02);
    extensionSim.update(0.02);

    inputs.rotationConnected = true;
    inputs.angle = Radians.of(angleSim.getAngularPositionRad());
    inputs.angularVelocity = RadiansPerSecond.of(angleSim.getAngularVelocityRadPerSec());
    inputs.angleAppliedVolts = Volts.of(angleAppliedVolts);
    inputs.angleCurrent = Amps.of(angleSim.getCurrentDrawAmps());

    inputs.extensionConnected = true;
    inputs.extensionLength = Meters.of(extensionSim.getAngularPositionRad());
    inputs.extensionLengthPerSec = MetersPerSecond.of(angleSim.getAngularVelocityRadPerSec());
    inputs.extensionAppliedVolts = Volts.of(extensionAppliedVolts);
    inputs.extensionCurrent = Amps.of(extensionSim.getCurrentDrawAmps());
  }

  @Override
  public void setAngleVoltage(double voltage) {
    angleClosedLoop = false;
    angleAppliedVolts = voltage;
  }

  @Override
  public void setAngle(Angle angle) {
    angleClosedLoop = true;
    // Must be in the same units (Radians) as what's used for the PID Controller calculate
    angleController.setSetpoint(angle.in(Radians));
  }

  @Override
  public void setLengthVoltage(double voltage) {
    extensionClosedLoop = false;
    extensionAppliedVolts = voltage;
  }

  @Override
  public void setLength(Distance length) {
    extensionClosedLoop = true;
    // Must be in the same units (Radians) as what's used for the PID Controller calculate. We're treating radians as
    // analogous to Meters in this sim, so they're still technically equivalent
    extensionController.setSetpoint(length.in(Meters));

  }
}
