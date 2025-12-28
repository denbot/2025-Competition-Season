package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autoCommands.BoathookCommands;
import frc.robot.commands.autoCommands.IntakeCommands;
import frc.robot.commands.autoCommands.OnTheFlyCommands;
import frc.robot.control.controllers.ButtonBoxController;
import frc.robot.control.controllers.DenbotXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LEDController;

import java.util.Optional;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

public class TeleopControl {
  private final Drive drive;
  private Command extendBoathook;
  private Command retractBoathook;
  private Command scorePrepCommand;
  private Command currentOnTheFlyCommand;

  public TeleopControl(
      EventLoop teleopEventLoop,
      DenbotXboxController driverController,
      ButtonBoxController buttonBoxController,
      Drive drive,
      BoathookCommands boathookCommands,
      IntakeCommands intakeCommands,
      OnTheFlyCommands onTheFlyCommands,
      LEDController ledController
  ) {
    this.drive = drive;

    Command setL1 = Commands.runOnce(() -> scorePrepCommand = intakeCommands.intakeL1Command());
    Command setL2 = Commands.runOnce(() -> {
          extendBoathook = boathookCommands.extendL2();
          retractBoathook = boathookCommands.retractL2();
          scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
        }
    );
    Command setL3 = Commands.runOnce(() -> {
          extendBoathook = boathookCommands.extendL3();
          retractBoathook = boathookCommands.retractL3();
          scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
        }
    );
    Command setL4 = Commands.runOnce(() -> {
          extendBoathook = boathookCommands.extendL4();
          retractBoathook = boathookCommands.retractL4();
          scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);
        }
    );

    extendBoathook = boathookCommands.extendL2();
    retractBoathook = boathookCommands.retractL2();
    scorePrepCommand = boathookCommands.handoffCommand(intakeCommands);

    currentOnTheFlyCommand = onTheFlyCommands.alignSixRight();


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
        .onTrue(
            Commands.runOnce(this::resetGyro, drive)
                .ignoringDisable(true)
        );

    driverController
        .x(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand.schedule())
                .alongWith(ledController.rainbow())
                .until(() -> !currentOnTheFlyCommand.isScheduled())
                .andThen(
                    ledController.temporary(Color.kYellow, Milliseconds.of(500))
                )
        );

    driverController
        .rightBumper(teleopEventLoop)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (!currentOnTheFlyCommand.isScheduled() || currentOnTheFlyCommand.isFinished()) {
                    extendBoathook.schedule();
                  }
                }
            )
        );

    driverController
        .rightTrigger(teleopEventLoop)
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
                .alongWith(
                    ledController.run(LEDPattern.solid(Color.kGreen).blink(Milliseconds.of(500)))
                )
                .andThen(ledController.fill(Color.kBlack))
        );

    driverController.povLeft(teleopEventLoop).onTrue(boathookCommands.microAdjustAngleBackward());
    driverController.povRight(teleopEventLoop).onTrue(boathookCommands.microAdjustAngleForward());
    driverController.povDown(teleopEventLoop).onTrue(boathookCommands.microAdjustExtensionBackward());
    driverController.povUp(teleopEventLoop).onTrue(boathookCommands.microAdjustExtensionForward());

    buttonBoxController
        .twoLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwoLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twoRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwoRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .fourLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignFourLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .fourRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignFourRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .sixLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignSixLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .sixRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignSixRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .eightLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignEightLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .eightRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignEightRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .tenLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTenLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .tenRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTenRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twelveLeftTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwelveLeft())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .twelveRightTrigger(teleopEventLoop)
        .onTrue(
            Commands.runOnce(() -> currentOnTheFlyCommand = onTheFlyCommands.alignTwelveRight())
                .alongWith(
                    ledController.temporary(Color.kRed, Milliseconds.of(250))
                )
        );

    buttonBoxController
        .L1Trigger(teleopEventLoop)
        .onTrue(
            setL1.alongWith(ledController.temporary(Color.kRed, Seconds.of(1)))
        );
    buttonBoxController.L2Trigger(teleopEventLoop).onTrue(setL2.alongWith(ledController.indicateL2()));
    buttonBoxController.L3Trigger(teleopEventLoop).onTrue(setL3.alongWith(ledController.indicateL3()));
    buttonBoxController.L4Trigger(teleopEventLoop).onTrue(setL4.alongWith(ledController.indicateL4()));

    // Clear Commands
    buttonBoxController
        .spearTrigger(teleopEventLoop)
        .onTrue(Commands.runOnce(() -> scorePrepCommand.schedule()));
  }

  private void resetGyro() {
    var alliance = DriverStation.getAlliance();
    boolean isFlipped = alliance.equals(Optional.of(DriverStation.Alliance.Red));
    Rotation2d rotation = isFlipped ? new Rotation2d(Math.PI) : new Rotation2d();
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), rotation));
  }
}
