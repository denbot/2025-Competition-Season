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
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoCommands.AutoRoutineBuilder;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.commands.elasticCommands.PreCheckTab;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.RumbleSubsystem;
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
  public Command extendBoathook;
  public Command retractBoathook;
  public Command scorePrepCommand;
  // permmanent

  private final Command pullInCoral;
  private final Command rejectCoral;
  private final Command setIntakeDown;
  private final Command setIntakeL1;

  private final Command microRotationAdjustForwards;
  private final Command microRotationAdjustBackwards;
  private final Command microExtensionAdjustInwards;
  private final Command microExtensionAdjustOutwards;

  public static Command currentOnTheFlyCommand;

  public BoathookCommands boathookCommands;
  public IntakeCommands intakeCommands;

  // each of these corresponds to a different button on the button board
  // these should set the pipeline to the side of the reef where the button is located
  // numbers correspond to clock faces with twelve being the back face of the reef

  private final Command SetL1 =
      Commands.runOnce(
          () -> {
            scorePrepCommand = intakeCommands.intakeL1Command();
          });
  private final Command SetL2 =
      Commands.runOnce(
          () -> {
            extendBoathook = boathookCommands.extendL2();
            retractBoathook = boathookCommands.retractL2();
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
          });
  private final Command SetL3 =
      Commands.runOnce(
          () -> {
            extendBoathook = boathookCommands.extendL3();
            retractBoathook = boathookCommands.retractL3();
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
          });
  private final Command SetL4 =
      Commands.runOnce(
          () -> {
            extendBoathook = boathookCommands.extendL4();
            retractBoathook = boathookCommands.retractL4();
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
          });

  public final AutoRoutineBuilder autoRoutineBuilder;

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

    intakeCommands = new IntakeCommands(intake);
    boathookCommands = new BoathookCommands(boathook);

    extendBoathook = boathookCommands.extendL2();
    retractBoathook = boathookCommands.retractL2();
    scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);

    pullInCoral = intakeCommands.runIntakeCommand();
    rejectCoral = intakeCommands.runRejectCommand();
    setIntakeDown = intakeCommands.intakeDownCommand();
    setIntakeL1 = intakeCommands.intakeL1Command();

    microRotationAdjustForwards = boathookCommands.MicroAdjustAngleForward();
    microRotationAdjustBackwards = boathookCommands.MicroAdjustAngleBackward();
    microExtensionAdjustInwards = boathookCommands.MicroAdjustExtensionBackward();
    microExtensionAdjustOutwards = boathookCommands.MicroAdjustExtensionForward();

    // onTheFlyAlignCommand = new OnTheFlyAlignCommand(drive);

    autoRoutineBuilder =
        new AutoRoutineBuilder()
            .addBuildingBlock(OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL2())
            .addBuildingBlock(OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL3())
            .addBuildingBlock(OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL4());

    rumblePresets = new RumblePresets(rumbleSubsystem);
    currentOnTheFlyCommand = OnTheFlyCommands.alignTwoLeft();

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

    controller.x().onTrue(Commands.runOnce(() -> currentOnTheFlyCommand.schedule()));

    controller.rightBumper().onTrue(Commands.runOnce(() -> extendBoathook.schedule()));
    controller.rightTrigger().onTrue(Commands.runOnce(() -> retractBoathook.schedule()));

    controller.leftBumper().whileTrue(rejectCoral);
    controller.leftTrigger().whileTrue(pullInCoral);
    controller
        .y()
        .onTrue(
            intake.getRotationAngle() == IntakeConstants.intakeL1Angle
                ? setIntakeDown
                : setIntakeL1);

    // boathook.setDefaultCommand(idleBoathook);
    controller.povLeft().onTrue(microRotationAdjustBackwards);
    controller.povRight().onTrue(microRotationAdjustForwards);
    controller.povDown().onTrue(microExtensionAdjustInwards);
    controller.povUp().onTrue(microExtensionAdjustOutwards);
    // controller.back().onTrue(Commands.runOnce(() -> m_orchestra.play()).ignoringDisable(true));
    // controller.leftStick().onTrue(Commands.runOnce(() ->
    // m_orchestra.stop()).ignoringDisable(true));

    configureAutoBuilderBindings();

    // TODO Consolidate operatorController 1 & 2 into One class

    operatorController1
        .button(1)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwelveLeft()));
    operatorController1
        .button(2)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwoRight()));
    operatorController1
        .button(3)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwoLeft()));
    operatorController1.button(4).onTrue(SetL4);
    operatorController1.button(5).onTrue(SetL3);
    operatorController1.button(6).onTrue(SetL2);
    operatorController1.button(7).onTrue(SetL1);
    operatorController1
        .button(8)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignFourRight()));
    operatorController1
        .button(11)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignFourLeft()));
    operatorController1
        .button(12)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignSixRight()));

    operatorController2
        .button(1)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwelverRight()));
    operatorController2
        .button(2)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTenLeft()));
    operatorController2
        .button(3)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTenRight()));
    operatorController2.button(4).onTrue(Commands.runOnce(() -> scorePrepCommand.schedule()));
    // operatorController2.button(5).onTrue(TODO);
    // operatorController2.button(6).onTrue(TODO);
    // operatorController2.button(7).onTrue(TODO);
    operatorController2
        .button(8)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignEightLeft()));
    operatorController2
        .button(11)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignEightRight()));
    operatorController2
        .button(12)
        .onTrue(Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignSixLeft()));
  }

  public void configureAutoBuilderBindings() {
    // Score L4 Bindings
    operatorController1
        .button(1)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController1
        .button(2)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController1
        .button(3)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController1
        .button(8)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController1
        .button(11)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController1
        .button(12)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));

    operatorController2
        .button(1)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelverRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController2
        .button(2)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController2
        .button(3)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController2
        .button(8)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController2
        .button(11)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL4()))
                .ignoringDisable(true));
    operatorController2
        .button(12)
        .and(operatorController1.button(4))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL4()))
                .ignoringDisable(true));

    // L3 Commands
    operatorController1
        .button(1)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController1
        .button(2)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController1
        .button(3)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController1
        .button(8)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController1
        .button(11)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController1
        .button(12)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));

    operatorController2
        .button(1)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelverRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController2
        .button(2)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController2
        .button(3)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController2
        .button(8)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController2
        .button(11)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL3()))
                .ignoringDisable(true));
    operatorController2
        .button(12)
        .and(operatorController1.button(5))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL3()))
                .ignoringDisable(true));

    // L2 Commands
    operatorController1
        .button(1)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController1
        .button(2)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController1
        .button(3)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController1
        .button(8)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController1
        .button(11)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController1
        .button(12)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));

    operatorController2
        .button(1)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTwelverRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController2
        .button(2)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController2
        .button(3)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController2
        .button(8)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController2
        .button(11)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
    operatorController2
        .button(12)
        .and(operatorController1.button(6))
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addBuildingBlock(
                            OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL2()))
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoRoutineBuilder.build();
  }
}
