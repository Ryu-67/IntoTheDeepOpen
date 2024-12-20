package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@Config
public class PivotCommand extends CommandBase {

    public static double degreesAllowed = 5;

    private MPPivot pivot;
    private double targetAngle = 0;

    boolean f = true;

    public PivotCommand(MPPivot pivot, double targetAngleDeg) {
        this.pivot = pivot; this.targetAngle = targetAngleDeg;
    }

    @Override
    public void execute() {
        if (f) {
            pivot.setTargetAngle(targetAngle);
            f = false;
        }
        pivot.update();
    }

    @Override
    public boolean isFinished() {
        return isPivotPast(Math.toDegrees(pivot.angle));
    }

    private boolean isPivotPast(double angle) {
        return Math.abs(targetAngle-angle) < degreesAllowed;
    }

}
