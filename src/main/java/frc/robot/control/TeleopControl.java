package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.control.controllers.ButtonBoxController;
import frc.robot.control.controllers.DenbotXboxController;
import frc.robot.game.ReefBranch;
import frc.robot.game.ReefLevel;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.subsystems.drive.Drive;
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

  public TeleopControl(
      EventLoop teleopEventLoop,
      DenbotXboxController driverController,
      ButtonBoxController buttonBoxController,
      Drive drive,
      Boathook boathook,
      BoathookCommands boathookCommands,
      IntakeCommands intakeCommands,
      OnTheFlyCommands onTheFlyCommands,
      LEDController ledController
  ) {
    SelectCommand<ReefLevel> scorePrepCommand = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, intakeCommands.intakeL1Command()),
            Map.entry(ReefLevel.L2, boathookCommands.handoffCommand(intakeCommands)),
            Map.entry(ReefLevel.L3, boathookCommands.handoffCommand(intakeCommands)),
            Map.entry(ReefLevel.L4, boathookCommands.handoffCommand(intakeCommands))
        ),
        () -> targetReefLevel
    );

    SelectCommand<ReefLevel> extendBoathook = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, Commands.idle(boathook)),
            Map.entry(ReefLevel.L2, boathookCommands.extendL2()),
            Map.entry(ReefLevel.L3, boathookCommands.extendL3()),
            Map.entry(ReefLevel.L4, boathookCommands.extendL4())
        ),
        () -> targetReefLevel
    );

    SelectCommand<ReefLevel> retractBoathook = new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ReefLevel.L1, Commands.idle(boathook)),
            Map.entry(ReefLevel.L2, boathookCommands.retractL2()),
            Map.entry(ReefLevel.L3, boathookCommands.retractL3()),
            Map.entry(ReefLevel.L4, boathookCommands.retractL4())
        ),
        () -> targetReefLevel
    );

    SelectCommand<ReefBranch> onTheFlyCommand = new SelectCommand<>(
        onTheFlyCommands.branchToAlignmentCommands(),
        () -> targetReefBranch
    );

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY() * (driverController.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -driverController.getLeftX() * (driverController.rightStick().getAsBoolean() ? 1 : 0.8),
            () -> -driverController.getRightX() * 0.8));

    // Lock to 0° when A button is held
    // TODO Lock this into rotating around the reef
    driverController
        .a(teleopEventLoop)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                Rotation2d::new));

    // Reset gyro to 0° when the Start button is pressed
    driverController
        .start(teleopEventLoop)
        .onTrue(Commands.runOnce(() -> resetGyro(drive), drive).ignoringDisable(true));

    driverController
        .x(teleopEventLoop)
        .onTrue(
            onTheFlyCommand
                .alongWith(ledController.rainbow())
                .andThen(ledController.temporary(Color.kYellow, Milliseconds.of(500)))
        );

    // We don't want to accidentally trigger until we're at the correct location around the reef
    BooleanSupplier onTheFlyIsNotRunning = onTheFlyCommand::isScheduled;

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
            intakeCommands.runRejectCommand()
                .alongWith(ledController.temporary(Color.kRed, Milliseconds.of(500)))
        );

    driverController
        .leftTrigger(teleopEventLoop)
        .whileTrue(
            intakeCommands.intakeDownCommand()
                .alongWith(intakeCommands.runIntakeCommand())
                .alongWith(ledController.temporary(Color.kGreen, Milliseconds.of(500)))
        );

    driverController
        .povLeft(teleopEventLoop)
        .onTrue(boathookCommands.microAdjustAngleBackward());

    driverController
        .povRight(teleopEventLoop)
        .onTrue(boathookCommands.microAdjustAngleForward());

    driverController
        .povDown(teleopEventLoop)
        .onTrue(boathookCommands.microAdjustExtensionBackward());

    driverController
        .povUp(teleopEventLoop)
        .onTrue(boathookCommands.microAdjustExtensionForward());

    buttonBoxController
        .buttonToReefBranchMap(teleopEventLoop)
        .forEach(
            (trigger, branch) -> trigger.onTrue(
                Commands.runOnce(() -> targetReefBranch = branch)
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
    buttonBoxController
        .spearTrigger(teleopEventLoop)
        .onTrue(scorePrepCommand);
  }

  private void resetGyro(Drive drive) {
    var alliance = DriverStation.getAlliance();
    boolean isFlipped = alliance.equals(Optional.of(DriverStation.Alliance.Red));
    Rotation2d rotation = isFlipped ? new Rotation2d(Math.PI) : new Rotation2d();
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), rotation));
  }
}
