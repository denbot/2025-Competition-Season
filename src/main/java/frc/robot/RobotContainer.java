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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoCommands.AutoRoutineBuilder;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.commands.elasticCommands.PreCheckTab;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ButtonBoxController;
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
  public Leds leds;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ButtonBoxController buttonBoxController = new ButtonBoxController();

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
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands, leds);
          });
  private final Command SetL3 =
      Commands.runOnce(
          () -> {
            extendBoathook = boathookCommands.extendL3();
            retractBoathook = boathookCommands.retractL3();
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands, leds);
          });
  private final Command SetL4 =
      Commands.runOnce(
          () -> {
            extendBoathook = boathookCommands.extendL4();
            retractBoathook = boathookCommands.retractL4();
            scorePrepCommand = boathookCommands.handoffCommand(intakeCommands, leds);
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
    leds = new Leds();

    intakeCommands = new IntakeCommands(intake);
    boathookCommands = new BoathookCommands(boathook);

    extendBoathook = boathookCommands.extendL2();
    retractBoathook = boathookCommands.retractL2();
    scorePrepCommand = boathookCommands.handoffCommand(intakeCommands, leds);

    pullInCoral = intakeCommands.runIntakeCommand();
    rejectCoral = intakeCommands.runRejectCommand();

    microRotationAdjustForwards = boathookCommands.MicroAdjustAngleForward();
    microRotationAdjustBackwards = boathookCommands.MicroAdjustAngleBackward();
    microExtensionAdjustInwards = boathookCommands.MicroAdjustExtensionBackward();
    microExtensionAdjustOutwards = boathookCommands.MicroAdjustExtensionForward();

    // onTheFlyAlignCommand = new OnTheFlyAlignCommand(drive);

    autoRoutineBuilder = new AutoRoutineBuilder(boathookCommands, intakeCommands);
    /* Test Auto Routine Builder Pattern
        .addBuildingBlock(OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL2())
        .addBuildingBlock(OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL3())
        .addBuildingBlock(OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL4());
    */

    rumblePresets = new RumblePresets(rumbleSubsystem);
    currentOnTheFlyCommand = OnTheFlyCommands.alignSixRight();

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

    preCheckTab =
        new PreCheckTab(
            controller, ButtonBoxController.controller1, ButtonBoxController.controller2);
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

    controller
        .x()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand.schedule())
                .alongWith(
                    Commands.runEnd(() -> leds.rainbow(), () -> leds.fullSolid(60, 255, 255))
                        .until(() -> !currentOnTheFlyCommand.isScheduled())));

    controller.rightBumper().onTrue(Commands.runOnce(() -> extendBoathook.schedule()));
    controller.rightTrigger().onTrue(Commands.runOnce(() -> retractBoathook.schedule()));

    controller
        .povUp()
        .onTrue(Commands.runOnce(() -> boathookCommands.MicroAdjustExtensionForward().schedule()));
    controller
        .povDown()
        .onTrue(Commands.runOnce(() -> boathookCommands.MicroAdjustExtensionBackward().schedule()));
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> boathookCommands.MicroAdjustAngleForward().schedule()));
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> boathookCommands.MicroAdjustAngleBackward().schedule()));

    controller
        .leftBumper()
        .whileTrue(
            rejectCoral.alongWith(
                Commands.runEnd(
                        () -> leds.flashSection(0, 21, 0, 255, 255, 0.25),
                        () -> leds.fullSolid(0, 0, 0))
                    .withTimeout(0.5)));
    controller
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(() -> intakeCommands.intakeDownCommand().schedule())
                .alongWith(pullInCoral)
                .alongWith(
                    Commands.runEnd(
                        () -> leds.flash(120, 255, 255, 0.5), () -> leds.fullSolid(0, 0, 0))));

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

    buttonBoxController
        .twoLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwoLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(15, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .twoRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwoRight())
                .alongWith(
                    Commands.run(() -> leds.flash(0, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .fourLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignFourLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(45, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .fourRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignFourRight())
                .alongWith(
                    Commands.run(() -> leds.flash(30, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .sixLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignSixLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(75, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .sixRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignSixRight())
                .alongWith(
                    Commands.run(() -> leds.flash(60, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .eightLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignEightLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(105, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .eightRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignEightRight())
                .alongWith(
                    Commands.run(() -> leds.flash(90, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .tenLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTenLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(135, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .tenRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTenRight())
                .alongWith(
                    Commands.run(() -> leds.flash(120, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .twelveLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwelveLeft())
                .alongWith(
                    Commands.run(() -> leds.flash(165, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));
    buttonBoxController
        .twelveRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = OnTheFlyCommands.alignTwelveRight())
                .alongWith(
                    Commands.run(() -> leds.flash(150, 255, 255, 0.25))
                        .withTimeout(0.5)
                        .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0)))));

    buttonBoxController
        .L1Trigger()
        .onTrue(
            SetL1.andThen(Commands.runOnce(() -> leds.fullSolid(150, 255, 255)))
                .andThen(new WaitCommand(1))
                .andThen(Commands.runOnce(() -> leds.fullSolid(0, 0, 0))));
    buttonBoxController.L2Trigger().onTrue(SetL2.alongWith(leds.indicateL2()));
    buttonBoxController.L3Trigger().onTrue(SetL3.alongWith(leds.indicateL3()));
    buttonBoxController.L4Trigger().onTrue(SetL4.alongWith(leds.indicateL4()));

    buttonBoxController
        .lollipopLeftTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addPickupPieceBlock(
                            OnTheFlyCommands.pickupLollipopLeft(intakeCommands)))
                .ignoringDisable(true));
    buttonBoxController
        .lollipopCenterTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addPickupPieceBlock(
                            OnTheFlyCommands.pickupLollipopCenter(intakeCommands)))
                .ignoringDisable(true));
    buttonBoxController
        .lollipopRightTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        autoRoutineBuilder.addPickupPieceBlock(
                            OnTheFlyCommands.pickupLollipopRight(intakeCommands)))
                .ignoringDisable(true));

    // Clear Commands
    buttonBoxController
        .spearTrigger()
        .onTrue(Commands.runOnce(() -> autoRoutineBuilder.clearCommands()).ignoringDisable(true))
        .onTrue(Commands.runOnce(() -> scorePrepCommand.schedule()));
  }

  public void configureAutoBuilderBindings() {
    // L4 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL4()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL4()));

    // L3 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL3()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL3()));

    // L2 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTenRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignEightRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignSixRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignFourRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL2()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                OnTheFlyCommands.alignTwoRight(), boathookCommands.scoreL2()));
  }

  public void assignButtonBinding(
      Trigger alignButton, Trigger scoreButton, Runnable buildingRunnable) {
    alignButton.and(scoreButton).onTrue(Commands.runOnce(buildingRunnable).ignoringDisable(true));
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
