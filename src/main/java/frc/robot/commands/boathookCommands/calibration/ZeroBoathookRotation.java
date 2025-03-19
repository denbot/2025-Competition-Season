package frc.robot.commands.boathookCommands.calibration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.util.Suppliers;
import frc.robot.util.elastic.Elastic;
import java.util.function.DoubleSupplier;

public class ZeroBoathookRotation extends Command {
  private static final double OFFSET_TO_ZERO = -.0115;
  private final Boathook boathook;
  private final Timer timer;
  private final Elastic.Tabs tab;
  private final DoubleSupplier magnetOffsetProperty;

  public ZeroBoathookRotation(Boathook boathook) {
    addRequirements(boathook);
    this.boathook = boathook;
    this.tab = Elastic.Tabs.BOATHOOK;
    this.timer = new Timer();
    magnetOffsetProperty = Suppliers.memoizeWithExpiration(boathook::rotationMagnetOffset, 5000);
  }

  @Override
  public void initialize() {
    this.timer.restart();
    this.boathook.setForceRotateIn();
    this.boathook.setRotationMagnetOffset(0);
  }

  @Override
  public void execute() {
    NetworkTable extension = tab.table().getSubTable("Zero Boathook Rotation");
    extension.getEntry("Min Limit").setBoolean(boathook.isExtenderAtMinimumLimit());
    extension.getEntry("Position").setDouble(boathook.getLength());
    extension.getEntry("CANcoder magnet offset").setDouble(magnetOffsetProperty.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    this.boathook.setNeutralRotate();

    if (interrupted) {
      return;
    }

    boathook.zeroRotationOffset(OFFSET_TO_ZERO);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }
}
