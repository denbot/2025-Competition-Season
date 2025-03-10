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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BoathookConstants;
import frc.robot.Constants.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.boathookCommands.BoathookExtendMotionPathCommand;
import frc.robot.commands.boathookCommands.BoathookIdleCommand;
import frc.robot.commands.boathookCommands.BoathookRetractMotionPathCommand;
import frc.robot.commands.boathookCommands.BoathookStabCommand;
import frc.robot.commands.boathookCommands.SetSetPointsCommand;
import frc.robot.commands.intakeCommands.*;
import frc.robot.commands.visionCommands.GoToReefCommand;
import frc.robot.commands.visionCommands.PipelineChange;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.boathook.Boathook;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandGenericHID operatorController1 = new CommandGenericHID(1);
  private final CommandGenericHID operatorController2 = new CommandGenericHID(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands
  private final GoToReefCommand reef;

  private final BoathookIdleCommand idleBoathook;
  private final BoathookExtendMotionPathCommand extendBoathook;
  private final BoathookRetractMotionPathCommand retractBoathook;
  private final BoathookStabCommand stabBoathook;

  private final StartIntake startIntake;
  private final StartIntake rejectIntake;
  private final FunnelIntake funnelIntake;
  private final StopIntake stopIntake;

  // each of these corresponds to a different button on the button board
  // these should set the pipeline to the side of the reef where the button is located
  // numbers correspond to clock faces with twelve being the back face of the reef
  private final PipelineChange twelveLeft = new PipelineChange(4, Direction.LEFT, 180);
  private final PipelineChange twelveRight = new PipelineChange(4, Direction.RIGHT, 180);

  private final PipelineChange tenLeft = new PipelineChange(5, Direction.LEFT, -120);
  private final PipelineChange tenRight = new PipelineChange(5, Direction.RIGHT, -120);

  private final PipelineChange eightLeft = new PipelineChange(6, Direction.LEFT, -60);
  private final PipelineChange eightRight = new PipelineChange(6, Direction.RIGHT, -60);

  private final PipelineChange sixLeft = new PipelineChange(1, Direction.LEFT, 0);
  private final PipelineChange sixRight = new PipelineChange(1, Direction.RIGHT, 0);

  private final PipelineChange fourLeft = new PipelineChange(2, Direction.LEFT, 60);
  private final PipelineChange fourRight = new PipelineChange(2, Direction.RIGHT, 60);

  private final PipelineChange twoLeft = new PipelineChange(3, Direction.LEFT, 120);
  private final PipelineChange twoRight = new PipelineChange(3, Direction.RIGHT, 120);

  private final SetSetPointsCommand L1 =
      new SetSetPointsCommand(
          BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION,
          BoathookConstants.L2_SCORE_ANGLE, BoathookConstants.IDLE_EXTENSION,
          BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION);
  private final SetSetPointsCommand L2 =
      new SetSetPointsCommand(
          BoathookConstants.IDLE_ANGLE, BoathookConstants.L2_EXTENSION,
          BoathookConstants.L2_SETUP_ANGLE, BoathookConstants.L2_EXTENSION,
          BoathookConstants.L2_SCORE_ANGLE, BoathookConstants.IDLE_EXTENSION);
  private final SetSetPointsCommand L3 =
      new SetSetPointsCommand(
          BoathookConstants.IDLE_ANGLE, BoathookConstants.L3_EXTENSION,
          BoathookConstants.L3_SETUP_ANGLE, BoathookConstants.L3_EXTENSION,
          BoathookConstants.L3_SCORE_ANGLE, BoathookConstants.L2_EXTENSION);
  private final SetSetPointsCommand L4 =
      new SetSetPointsCommand(
          BoathookConstants.IDLE_ANGLE, BoathookConstants.L4_EXTENSION,
          BoathookConstants.L4_SETUP_ANGLE, BoathookConstants.L3_EXTENSION,
          BoathookConstants.IDLE_ANGLE, BoathookConstants.IDLE_EXTENSION);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

    reef = new GoToReefCommand(drive);
    idleBoathook = new BoathookIdleCommand(boathook);
    extendBoathook = new BoathookExtendMotionPathCommand(boathook);
    retractBoathook = new BoathookRetractMotionPathCommand(boathook);
    stabBoathook = new BoathookStabCommand(boathook, intake);
    startIntake = new StartIntake(intake, 2);
    rejectIntake = new StartIntake(intake, -2);
    funnelIntake = new FunnelIntake(intake);
    stopIntake = new StopIntake(intake);

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

    controller.leftBumper().whileTrue(startIntake);
    controller.leftBumper().onFalse(stopIntake);
    controller.leftTrigger().whileTrue(rejectIntake);
    controller.leftTrigger().onFalse(stopIntake);
    controller.y().onTrue(stopIntake);
    controller.x().onTrue(funnelIntake);

    // boathook.setDefaultCommand(idleBoathook);
    // controller.rightBumper().onTrue(extendBoathook);
    // controller.rightTrigger().onTrue(retractBoathook);

    operatorController1.button(1).onTrue(twelveLeft);
    operatorController1.button(2).onTrue(twoRight);
    operatorController1.button(3).onTrue(twoLeft);
    operatorController1.button(4).onTrue(L4);
    operatorController1.button(5).onTrue(L3);
    operatorController1.button(6).onTrue(L2);
    operatorController1.button(7).onTrue(L1);
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
