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
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OrchestraPlayer;
import frc.robot.commands.autoCommands.AutoRoutineBuilder;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.commands.elasticCommands.PreCheckTab;
import frc.robot.commands.status.AllianceStatus;
import frc.robot.generated.TunerConstants;
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
import frc.robot.util.ButtonBoxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Intake intake;
  private final Boathook boathook;
  private final RumbleSubsystem rumbleSubsystem;
  private final AutoRoutineBuilder autoRoutineBuilder;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final ButtonBoxController buttonBoxController = new ButtonBoxController();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public final PreCheckTab preCheckTab;

  private final LEDController ledController;
  // Commands
  private Command extendBoathook;

  private Command retractBoathook;
  private Command scorePrepCommand;
  private BoathookCommands boathookCommands;
  private IntakeCommands intakeCommands;
  private final OnTheFlyCommands onTheFlyCommands;
  private Command currentOnTheFlyCommand;

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

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
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
    ledController = new LEDController(21);

    intakeCommands = new IntakeCommands(intake);
    boathookCommands = new BoathookCommands(boathook, ledController);
    onTheFlyCommands = new OnTheFlyCommands(boathookCommands, drive);

    extendBoathook = boathookCommands.extendL2();
    retractBoathook = boathookCommands.retractL2();
    scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);

    autoRoutineBuilder = new AutoRoutineBuilder(boathookCommands, intakeCommands);
    SmartDashboard.putStringArray("Auto Routine List", autoRoutineBuilder.getCommandStrings());

    currentOnTheFlyCommand = onTheFlyCommands.alignSixRight();

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

    OrchestraPlayer orchestraPlayer = new OrchestraPlayer(
        controller,
        intake,
        boathook
    );
    autoChooser.addOption("Music Player", orchestraPlayer);

    // Configure the button bindings
    configureButtonBindings();

    preCheckTab =
        new PreCheckTab(
            controller::isConnected,
            buttonBoxController::isControllerOneConnected,
            buttonBoxController::isControllerTwoConnected
        );

    // TODO Add a SendableChooser wrapper that automatically runs our status commands
    new AllianceStatus(ledController).schedule();
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
            () -> -controller.getLeftY() * (controller.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -controller.getLeftX() * (controller.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -controller.getRightX() * 0.8));

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

    // Reset gyro to 0° when the Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(this::resetGyro, drive)
                .ignoringDisable(true)
        );

    controller
        .x()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand.schedule())
                .alongWith(ledController.rainbow())
                .until(() -> !currentOnTheFlyCommand.isScheduled())
                .andThen(
                    ledController.temporary(Color.kYellow, Milliseconds.of(500))
                )
        );

    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!currentOnTheFlyCommand.isScheduled() || currentOnTheFlyCommand.isFinished()) {
                    extendBoathook.schedule();
                  }
                }
            )
        );

    controller
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (!currentOnTheFlyCommand.isScheduled() || currentOnTheFlyCommand.isFinished()) {
                        retractBoathook.schedule();
                      }
                    }
                )
                .until(() -> !retractBoathook.isScheduled())
                .andThen(ledController.fill(Color.kBlack))
        );

    controller
        .leftBumper()
        .whileTrue(
            intakeCommands.runRejectCommand()
                .alongWith(ledController.temporary(Color.kRed, Milliseconds.of(500)))
        );

    controller
        .leftTrigger()
        .whileTrue(
            intakeCommands.intakeDownCommand()
                .alongWith(intakeCommands.runIntakeCommand())
                .alongWith(
                    ledController.run(LEDPattern.solid(Color.kGreen).blink(Milliseconds.of(500)))
                )
                .andThen(ledController.fill(Color.kBlack))
        );

    controller.povLeft().onTrue(boathookCommands.microAdjustAngleBackward());
    controller.povRight().onTrue(boathookCommands.microAdjustAngleForward());
    controller.povDown().onTrue(boathookCommands.microAdjustExtensionBackward());
    controller.povUp().onTrue(boathookCommands.microAdjustExtensionForward());

    configureAutoBuilderBindings();

    buttonBoxController
        .twoLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwoLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twoRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwoRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .fourLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignFourLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .fourRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignFourRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .sixLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignSixLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .sixRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignSixRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .eightLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignEightLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .eightRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignEightRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .tenLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTenLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .tenRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTenRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twelveLeftTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwelveLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twelveRightTrigger()
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwelveRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .L1Trigger()
        .onTrue(
            SetL1.alongWith(ledController.temporary(Color.kRed, Seconds.of(1)))
        );
    buttonBoxController.L2Trigger().onTrue(SetL2.alongWith(ledController.indicateL2()));
    buttonBoxController.L3Trigger().onTrue(SetL3.alongWith(ledController.indicateL3()));
    buttonBoxController.L4Trigger().onTrue(SetL4.alongWith(ledController.indicateL4()));

    buttonBoxController
        .lollipopLeftTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> autoRoutineBuilder.addPickupPieceBlock(onTheFlyCommands.pickupLollipopLeft(intakeCommands))
                )
                .ignoringDisable(true)
        );

    buttonBoxController
        .lollipopCenterTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> autoRoutineBuilder.addPickupPieceBlock(onTheFlyCommands.pickupLollipopCenter(intakeCommands))
                )
                .ignoringDisable(true)
        );

    buttonBoxController
        .lollipopRightTrigger()
        .onTrue(
            Commands.runOnce(
                    () -> autoRoutineBuilder.addPickupPieceBlock(onTheFlyCommands.pickupLollipopRight(intakeCommands))
                )
                .ignoringDisable(true)
        );

    // Clear Commands
    buttonBoxController
        .spearTrigger()
        .onTrue(Commands.runOnce(autoRoutineBuilder::clearCommands).ignoringDisable(true))
        .onTrue(Commands.runOnce(() -> scorePrepCommand.schedule()));
  }

  private void resetGyro() {
    var alliance = DriverStation.getAlliance();
    boolean isFlipped = alliance.equals(Optional.of(DriverStation.Alliance.Red));
    Rotation2d rotation = isFlipped ? new Rotation2d(Math.PI) : new Rotation2d();
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), rotation));
  }

  private void configureAutoBuilderBindings() {
    // L4 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourLeft(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourRight(), boathookCommands.scoreL4()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL4()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L4Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoRight(), boathookCommands.scoreL4()));

    // L3 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourLeft(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourRight(), boathookCommands.scoreL3()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL3()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L3Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoRight(), boathookCommands.scoreL3()));

    // L2 Bindings
    assignButtonBinding(
        buttonBoxController.twelveLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twelveRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwelveRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.tenLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.tenRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTenRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.eightLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.eightRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignEightRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.sixLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.sixRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignSixRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.fourLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourLeft(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.fourRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignFourRight(), boathookCommands.scoreL2()));

    assignButtonBinding(
        buttonBoxController.twoLeftTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoLeft(), boathookCommands.scoreL2()));
    assignButtonBinding(
        buttonBoxController.twoRightTrigger(),
        buttonBoxController.L2Trigger(),
        () ->
            autoRoutineBuilder.addBuildingBlock(
                onTheFlyCommands.alignTwoRight(), boathookCommands.scoreL2()));
  }

  private void assignButtonBinding(
      Trigger alignButton, Trigger scoreButton, Runnable buildingRunnable) {
    alignButton.and(scoreButton).onTrue(Commands.runOnce(buildingRunnable).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // TODO We will get back to this one running the motion path is an auto option
//    return autoRoutineBuilder.build();
  }
}
