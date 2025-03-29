package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftTolCommand extends CommandBase {

    private double tolerance = 0;
    private Lift lift;

    public LiftTolCommand(double tol, Lift lift) {
        this.tolerance = tol;
        this.lift = lift;
    }

    private boolean first = true;

    @Override
    public void execute() {
        if (first) {
            lift.setPIDTolerance(tolerance);
        }
    }

    @Override
    public boolean isFinished() {
        return  true;
    }

}
