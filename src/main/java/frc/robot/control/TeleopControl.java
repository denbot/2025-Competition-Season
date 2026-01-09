package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.control.controllers.ButtonBoxController;
import frc.robot.control.controllers.DenbotXboxController;
import frc.robot.game.ReefBranch;
import frc.robot.game.ReefLevel;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDController;

import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

public class TeleopControl {
  // Chosen by fair dice roll. https://xkcd.com/221/
  private ReefLevel targetReefLevel = ReefLevel.L2;
  private ReefBranch targetReefBranch = ReefBranch.SIX_RIGHT;
  private Command onTheFlyCommand;

  public TeleopControl(
      EventLoop teleopEventLoop,
      DenbotXboxController driverController,
      ButtonBoxController buttonBoxController,
      Drive drive,
      Boathook boathook,
      Intake intake,
      OnTheFlyCommands onTheFlyCommands,
      LEDController ledController
  ) {
    SelectCommand<ReefLevel> scorePrepCommand = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, intake.intakeL1Command()),
            Map.entry(ReefLevel.L2, boathook.handoffCommand(intake)),
            Map.entry(ReefLevel.L3, boathook.handoffCommand(intake)),
            Map.entry(ReefLevel.L4, boathook.handoffCommand(intake))
        ),
        () -> targetReefLevel
    );

    SelectCommand<ReefLevel> extendBoathook = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, Commands.idle(boathook)),
            Map.entry(ReefLevel.L2, boathook.extendL2()),
            Map.entry(ReefLevel.L3, boathook.extendL3()),
            Map.entry(ReefLevel.L4, boathook.extendL4())
        ),
        () -> targetReefLevel
    );

    SelectCommand<ReefLevel> retractBoathook = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, Commands.idle(boathook)),
            Map.entry(ReefLevel.L2, boathook.retractL2()),
            Map.entry(ReefLevel.L3, boathook.retractL3()),
            Map.entry(ReefLevel.L4, boathook.retractL4())
        ),
        () -> targetReefLevel
    );

    onTheFlyCommand = onTheFlyCommands.getAutoAlignCommand(ReefBranch.TWELVE_LEFT);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY() * (driverController.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -driverController.getLeftX() * (driverController.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -driverController.getRightX() * 0.8)
    );

    // Orbit the reef when A button is held
    driverController
        .a(teleopEventLoop)
        .whileTrue(
            onTheFlyCommands.orbitReef(
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX()
            )
            .alongWith(Commands.repeatingSequence(
                ledController.fill(Color.kRed),
                new WaitCommand(0.25),
                ledController.fill(Color.kBlue),
                new WaitCommand(0.25)
            )));

    // Reset gyro to 0° when the Start button is pressed
    driverController
        .start(teleopEventLoop)
        .onTrue(Commands.runOnce(() -> resetGyro(drive), drive).ignoringDisable(true));

    // We don't want to accidentally trigger until we're at the correct location around the reef
    BooleanSupplier onTheFlyIsNotRunning = () -> !onTheFlyCommand.isScheduled();

    driverController
        .x(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> onTheFlyCommand.schedule())
                .andThen(Commands.repeatingSequence(
                    ledController.fill(Color.kOrange),
                    new WaitCommand(0.5),
                    ledController.fill(Color.kWhite),
                    new WaitCommand(0.5)
                ))
                .until(onTheFlyIsNotRunning)
                .andThen(ledController.temporary(Color.kGreen, Milliseconds.of(500)))
        );

    driverController
        .rightBumper(teleopEventLoop)
        .and(onTheFlyIsNotRunning)
        .onTrue(extendBoathook);

    driverController
        .rightTrigger(teleopEventLoop)
        .and(onTheFlyIsNotRunning)
        .onTrue(retractBoathook);

    driverController
        .leftBumper(teleopEventLoop)
        .whileTrue(
            intake.runRejectCommand()
                .alongWith(ledController.temporary(Color.kRed, Milliseconds.of(500)))
        );

    driverController
        .leftTrigger(teleopEventLoop)
        .whileTrue(
            intake.intakeDownCommand()
                .alongWith(ledController.temporary(Color.kGreen, Milliseconds.of(500)))
                .andThen(intake.runIntakeCommand())
                .alongWith(Commands.repeatingSequence(
                    ledController.fill(Color.kBlue),
                    new WaitCommand(0.5),
                    ledController.fill(Color.kWhite),
                    new WaitCommand(0.5)))
        );

    driverController
        .povLeft(teleopEventLoop)
        .onTrue(boathook.microAdjustAngleBackward());

    driverController
        .povRight(teleopEventLoop)
        .onTrue(boathook.microAdjustAngleForward());

    driverController
        .povDown(teleopEventLoop)
        .onTrue(boathook.microAdjustExtensionBackward());

    driverController
        .povUp(teleopEventLoop)
        .onTrue(boathook.microAdjustExtensionForward());

    buttonBoxController
        .buttonToReefBranchMap(teleopEventLoop)
        .forEach(
            (trigger, branch) -> trigger.onTrue(
                Commands.runOnce(() -> {
                    targetReefBranch = branch;
                    onTheFlyCommand = onTheFlyCommands.getAutoAlignCommand(targetReefBranch);})
                    .alongWith(
                        ledController.temporary(Color.kRed, Milliseconds.of(250))
                    )
            )
        );

    Map<ReefLevel, Command> ledIndicationMap = Map.ofEntries(
        Map.entry(ReefLevel.L1, ledController.temporary(Color.kRed, Seconds.of(1))),
        Map.entry(ReefLevel.L2, ledController.indicateL2()),
        Map.entry(ReefLevel.L3, ledController.indicateL3()),
        Map.entry(ReefLevel.L4, ledController.indicateL4())
    );

    buttonBoxController
        .buttonToReefLevelMap(teleopEventLoop)
        .forEach(
            (trigger, level) -> trigger.onTrue(
                Commands.runOnce(() -> targetReefLevel = level)
                    .alongWith(ledIndicationMap.get(level))
            )
        );

    // TODO This should probably be under drive control and automatic on intake
    driverController
        .y(teleopEventLoop)
        .onTrue(scorePrepCommand);
  }

  private void resetGyro(Drive drive) {
    var alliance = DriverStation.getAlliance();
    boolean isFlipped = alliance.equals(Optional.of(DriverStation.Alliance.Red));
    Rotation2d rotation = isFlipped ? new Rotation2d(Math.PI) : new Rotation2d();
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), rotation));
  }
}
