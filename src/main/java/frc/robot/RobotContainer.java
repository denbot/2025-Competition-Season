// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OrchestraPlayer;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.elasticCommands.PreCheckTab;
import frc.robot.commands.status.AllianceStatus;
import frc.robot.control.AutoSequenceUserControl;
import frc.robot.control.controllers.DenbotXboxController;
import frc.robot.control.TeleopControl;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.boathook.BoathookIO;
import frc.robot.subsystems.boathook.BoathookIOSim;
import frc.robot.subsystems.boathook.BoathookIOTalonFX;
import frc.robot.subsystems.led.LEDController;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.control.controllers.ButtonBoxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final PreCheckTab preCheckTab;
  private final Intake intake;
  private final Boathook boathook;
  private final RumbleSubsystem rumbleSubsystem;

  // Controller
  private final DenbotXboxController driverController = new DenbotXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final ButtonBoxController buttonBoxController = new ButtonBoxController();

  // Event loops depending on the robot state
  private final EventLoop disabledEventLoop = new EventLoop();
  private final EventLoop teleopEventLoop = new EventLoop();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands
  private final BoathookCommands boathookCommands;
  private final OnTheFlyCommands onTheFlyCommands;

  // Direct control over the LEDs
  private final LEDController ledController;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
        );

        boathook = new Boathook(new BoathookIOTalonFX());
        intake = new Intake(new IntakeIOReal());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight)
        );

        boathook = new Boathook(new BoathookIOSim());
        intake = new Intake(new IntakeIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {}
        );

        boathook = new Boathook(new BoathookIO() {});
        intake = new Intake(new IntakeIO() {});

        break;
    }

    rumbleSubsystem = new RumbleSubsystem(driverController);
    ledController = new LEDController(21);

    boathookCommands = new BoathookCommands(boathook, ledController);
    onTheFlyCommands = new OnTheFlyCommands(intake, boathookCommands, drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutoRoutines();

    // Configure the button bindings
    configureButtonBindings();

    preCheckTab = new PreCheckTab(
        driverController::isConnected,
        buttonBoxController::isControllerOneConnected,
        buttonBoxController::isControllerTwoConnected
    );

    // TODO Add a SendableChooser wrapper that automatically runs our status commands
    new AllianceStatus(ledController).schedule();

    // Depending on the mode, determine which button set is active. We could have set the active button loop instead,
    // but that precludes the option of having buttons enabled regardless of the mode in use.
    CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
      if (DriverStation.isDisabled()) {
        disabledEventLoop.poll();
      } else if (DriverStation.isTeleop()) {
        teleopEventLoop.poll();
      }
    });
  }

  /**
   * Registers all autonomous routines and characterization commands to the dashboard chooser.
   */
  private void configureAutoRoutines() {
    var orchestraPlayer = new OrchestraPlayer(
        driverController,
        boathook
    );
    autoChooser.addOption("Music Player", orchestraPlayer);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Configures the button bindings for various robot states.
   *
   * <p>Control logic is delegated to dedicated classes (e.g., {@code TeleopControl}) to keep
   * this file manageable and focused on high-level structure.
   *
   * <p>We use multiple {@link EventLoop} instances to ensure that input triggers are only
   * processed during appropriate robot modes (e.g., Disabled or Teleop), preventing
   * accidental command execution.
   */
  private void configureButtonBindings() {
    // Allow the operator to enter an auto using the button box.
    var autoSequenceUserSetup = new AutoSequenceUserControl(
        disabledEventLoop,
        buttonBoxController,
        boathookCommands,
        intake,
        onTheFlyCommands
    );

    // Add it as the default option in our auto chooser as we want to ensure it's set up correctly for matches without
    // having to check the auto chooser dropdown.
    autoChooser.addDefaultOption("Robot On The Fly Auto", autoSequenceUserSetup.runProgrammedSequence());

    // Since this is just for setting up triggers for the event loop, we don't need to do anything but create the object.
    // All other interactions will be done through teleopEventLoop
    new TeleopControl(
        teleopEventLoop,
        driverController,
        buttonBoxController,
        drive,
        boathook,
        boathookCommands,
        intake,
        onTheFlyCommands,
        ledController
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
