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

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.boathookCommands.BoathookExtendMotionPathCommand;
import frc.robot.commands.boathookCommands.BoathookRetractMotionPathCommand;
import frc.robot.commands.boathookCommands.HandoffCommand;
import frc.robot.commands.boathookCommands.SetLevelCommand;
import frc.robot.commands.elasticCommands.PreCheckTab;
import frc.robot.commands.intakeCommands.*;
import frc.robot.commands.visionCommands.GoToReefCommand;
import frc.robot.commands.visionCommands.TargetChange;
import frc.robot.game.ReefAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.boathook.Boathook.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
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
  public final Intake intake;
  public final Boathook boathook;
  public final RumbleSubsystem rumbleSubsystem;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandGenericHID operatorController1 = new CommandGenericHID(1);
  private final CommandGenericHID operatorController2 = new CommandGenericHID(2);

  // Dashboard inputs
  public DSControlWord controlWord = new DSControlWord();
  private final LoggedDashboardChooser<Command> autoChooser;
  public Orchestra m_orchestra = new Orchestra();

  // Commands
  private final GoToReefCommand reef;
  private final BoathookExtendMotionPathCommand extendBoathook;
  private final BoathookRetractMotionPathCommand retractBoathook;
  private final HandoffCommand stabBoathook;

  private final RunIntakeCommand pullInCoral;
  private final RunIntakeCommand rejectCoral;

  // each of these corresponds to a different button on the button board
  // these should set the pipeline to the side of the reef where the button is located
  // numbers correspond to clock faces with twelve being the back face of the reef
  private final TargetChange twelveLeft = new TargetChange(Direction.LEFT, 180, ReefAprilTag.TWELVE);
  private final TargetChange twelveRight =
      new TargetChange(Direction.RIGHT, 180, ReefAprilTag.TWELVE);

  private final TargetChange tenLeft = new TargetChange(Direction.LEFT, -120, ReefAprilTag.TEN);
  private final TargetChange tenRight = new TargetChange(Direction.RIGHT, -120, ReefAprilTag.TEN);

  private final TargetChange eightLeft = new TargetChange(Direction.LEFT, -60, ReefAprilTag.EIGHT);
  private final TargetChange eightRight = new TargetChange(Direction.RIGHT, -60, ReefAprilTag.EIGHT);

  private final TargetChange sixLeft = new TargetChange(Direction.LEFT, 0, ReefAprilTag.SIX);
  private final TargetChange sixRight = new TargetChange(Direction.RIGHT, 0, ReefAprilTag.SIX);

  private final TargetChange fourLeft = new TargetChange(Direction.LEFT, 60, ReefAprilTag.FOUR);
  private final TargetChange fourRight = new TargetChange(Direction.RIGHT, 60, ReefAprilTag.FOUR);

  private final TargetChange twoLeft = new TargetChange(Direction.LEFT, 120, ReefAprilTag.TWO);
  private final TargetChange twoRight = new TargetChange(Direction.RIGHT, 120, ReefAprilTag.TWO);

  private final SetLevelCommand SetL1 = new SetLevelCommand(Level.L1);
  private final SetLevelCommand SetL2 = new SetLevelCommand(Level.L2);
  private final SetLevelCommand SetL3 = new SetLevelCommand(Level.L3);
  private final SetLevelCommand SetL4 = new SetLevelCommand(Level.L4);

  public final RumblePresets rumblePresets;

  public final PreCheckTab preCheckTab;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    intake = new Intake();
    boathook = new Boathook();
    rumbleSubsystem = new RumbleSubsystem(controller);

    reef = new GoToReefCommand(drive);
    extendBoathook = new BoathookExtendMotionPathCommand(boathook);
    retractBoathook = new BoathookRetractMotionPathCommand(boathook);
    stabBoathook = new HandoffCommand(boathook, intake);

    pullInCoral =
        new RunIntakeCommand(intake, RunIntakeCommand.Direction.Intake, boathook, controlWord);
    rejectCoral =
        new RunIntakeCommand(intake, RunIntakeCommand.Direction.Eject, boathook, controlWord);

    rumblePresets = new RumblePresets(rumbleSubsystem);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // Configure the button bindings
    configureButtonBindings();

    preCheckTab = new PreCheckTab(controller, operatorController1, operatorController2);
    preCheckTab.schedule();

    // Attempt to load the chrp
    var status = m_orchestra.loadMusic("OceanMan.chrp");

    if (!status.isOK()) {
      // log error
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // TODO Lock this into rotating around the reef
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                Rotation2d::new));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                      Rotation2d rotation = isFlipped ? new Rotation2d(Math.PI) : new Rotation2d();
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), rotation));
                    },
                    drive)
                .ignoringDisable(true));

    controller.b().onTrue(reef);

    controller.leftBumper().whileTrue(pullInCoral);
    controller.leftTrigger().whileTrue(rejectCoral);

    // boathook.setDefaultCommand(idleBoathook);
    controller.rightBumper().onTrue(extendBoathook);
    controller.rightTrigger().onTrue(retractBoathook);

    // controller.back().onTrue(Commands.runOnce(() -> m_orchestra.play()).ignoringDisable(true));
    // controller.leftStick().onTrue(Commands.runOnce(() ->
    // m_orchestra.stop()).ignoringDisable(true));

    operatorController1.button(1).onTrue(twelveLeft);
    operatorController1.button(2).onTrue(twoRight);
    operatorController1.button(3).onTrue(twoLeft);
    operatorController1.button(4).onTrue(SetL4);
    operatorController1.button(5).onTrue(SetL3);
    operatorController1.button(6).onTrue(SetL2);
    operatorController1.button(7).onTrue(SetL1);
    operatorController1.button(8).onTrue(fourRight);
    operatorController1.button(11).onTrue(fourLeft);
    operatorController1.button(12).onTrue(sixRight);

    operatorController2.button(1).onTrue(twelveRight);
    operatorController2.button(2).onTrue(tenLeft);
    operatorController2.button(3).onTrue(tenRight);
    operatorController2.button(4).onTrue(stabBoathook);
    // operatorController2.button(5).onTrue(TODO);
    // operatorController2.button(6).onTrue(TODO);
    // operatorController2.button(7).onTrue(TODO);
    operatorController2.button(8).onTrue(eightLeft);
    operatorController2.button(11).onTrue(eightRight);
    operatorController2.button(12).onTrue(sixLeft);
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
