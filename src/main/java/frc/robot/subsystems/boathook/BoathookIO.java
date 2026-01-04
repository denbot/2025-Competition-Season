package frc.robot.subsystems.boathook;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface BoathookIO {
  /**
   * Updates the input state of the boathook mechanism by providing new sensor
   * and motor data.
   *
   * @param inputs An instance of BoathookIOInputs containing the current state of
   *               rotation and extension components.
   */
  default void updateInputs(BoathookIOInputs inputs) {
  }

  /**
   * Sets the voltage to be applied to control the angle of the boathook mechanism's
   * rotation component. This method adjusts the input voltage supplied to the motor
   * responsible for angular movement.
   *
   * @param voltage The voltage to be applied, specified in volts. Positive or negative
   *                values determine the direction of the rotation, while the magnitude
   *                influences the rotational speed.
   */
  default void setAngleVoltage(double voltage) {
  }

  default void setAnglePosition(Angle angle) {
  }

  default void setLengthVoltage(double voltage) {
  }

  default void setLengthPosition(Distance length) {
  }

  @AutoLog
  class BoathookIOInputs {
    public boolean rotationConnected = false;
    public Angle angle = Radians.zero();
    public AngularVelocity angularVelocity = RadiansPerSecond.zero();
    public Voltage angleAppliedVolts = Volts.zero();
    public Current angleCurrent = Amps.zero();

    public boolean extensionConnected = false;
    public Distance extensionLength = Meter.zero();
    public LinearVelocity extensionLengthPerSec = MetersPerSecond.zero();
    public Voltage extensionAppliedVolts = Volts.zero();
    public Current extensionCurrent = Amps.zero();
  }
}
