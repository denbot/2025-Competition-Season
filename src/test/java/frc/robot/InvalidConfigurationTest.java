package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.Test;

/**
 * This test verifies any static configurations set up in the project. Each test verifies a specific aspect of the
 * configuration. This is useful as it saves runtime cost of checking configurations that can be checked on the host
 * build computer.
 */
public class InvalidConfigurationTest {
  /**
   * Because we're using an AdvantageKit template, they only supported the TalonFX_Integrated drivers. This test ensures
   * that the codebase is using those drivers.
   */
  @Test
  void testSwerveMotorTypesAreTalonFXIntegrated() {
    var modules =
        new SwerveModuleConstants[]{
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated) {
        fail(
            "You are using an unsupported swerve configuration, which this template does not support without manual " +
                "customization. The 2025 release of Phoenix supports some swerve configurations which were not " +
                "available during 2025 beta testing, preventing any development and support from the AdvantageKit " +
                "developers."
        );
      }
    }
  }
}
