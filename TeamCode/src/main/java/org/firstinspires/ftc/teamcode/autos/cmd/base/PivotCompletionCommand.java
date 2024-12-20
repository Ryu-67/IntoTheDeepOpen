package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@Config
public class PivotCompletionCommand extends CommandBase {

    public static double degreesAllowed = 5, percentRequired = 0.8;

    private MPPivot pivot;
    private double targetAngle = 0, adjustedAllowance;

    boolean f = true;

    public PivotCompletionCommand(MPPivot pivot, double targetAngleDeg) {
        this.pivot = pivot; this.targetAngle = targetAngleDeg;
        adjustedAllowance = (percentRequired*targetAngleDeg);
    }

    @Override
    public void execute() {
        if (f) {
            pivot.setTargetAngle(targetAngle);
            f = false;
        }
    }

    @Override
    public boolean isFinished() {
        return isPivotPast(Math.toDegrees(pivot.angle));
    }

    private boolean isPivotPast(double angle) {
        return Math.abs(adjustedAllowance-angle) < degreesAllowed;
    }

}
