package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0);
    }

    public void setLedMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }
}