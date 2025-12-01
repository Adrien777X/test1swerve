package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends Command {

    private final SwerveSubsystem swerve;
    private final Limelight limelight;

    private final PIDController rotPID =
        new PIDController(0.03, 0.0, 0.002); // tune this

    public AutoAlign(SwerveSubsystem swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;

        rotPID.setTolerance(1.5); // degrees
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        limelight.setLedMode(3); // LED ON
    }

    @Override
    public void execute() {

        if (!limelight.hasTarget()) {
            // stop rot
            swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            return;
        }

        double error = limelight.getTx();
        double rot = rotPID.calculate(error, 0);

        // SwerveLib expects rot in rad/s
        rot = Math.max(-1.5, Math.min(1.5, rot));

        // only rotate — no X/Y movement
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, rot));
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLedMode(1); // LED OFF
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return limelight.hasTarget() && rotPID.atSetpoint();
    }
}