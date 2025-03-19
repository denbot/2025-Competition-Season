package frc.robot.commands.boathookCommands.calibration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.boathook.Boathook;
import frc.robot.util.Suppliers;
import frc.robot.util.elastic.Elastic;
import java.util.function.DoubleSupplier;

public class ZeroBoathookExtension extends Command {
  private final Boathook boathook;
  private final Elastic.Tabs tab;
  private final DoubleSupplier magnetOffsetProperty;

  public ZeroBoathookExtension(Boathook boathook) {
    addRequirements(boathook);
    this.boathook = boathook;
    this.tab = Elastic.Tabs.BOATHOOK;
    magnetOffsetProperty = Suppliers.memoizeWithExpiration(boathook::extensionMagnetOffset, 5000);
  }

  @Override
  public void initialize() {
    boathook.setMagnetOffset(0);
    if (boathook.isExtenderAtMinimumLimit()) {
      boathook.setBrakeExtender();
    } else {
      boathook.setSlowlyRetract();
    }
  }

  @Override
  public void execute() {
    NetworkTable extension = tab.table().getSubTable("Zero Boathook Extension");
    extension.getEntry("Min Limit").setBoolean(boathook.isExtenderAtMinimumLimit());
    extension.getEntry("Position").setDouble(boathook.getLength());
    extension.getEntry("CANcoder magnet offset").setDouble(magnetOffsetProperty.getAsDouble());

    if (!boathook.isExtenderAtMinimumLimit()) {
      return;
    }

    boathook.setBrakeExtender();
    boathook.zeroExtensionOffset();
    this.cancel();
  }

  @Override
  public void end(boolean interrupted) {
    boathook.setNeutralExtender();
  }
}
