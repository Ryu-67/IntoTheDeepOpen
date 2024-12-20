package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftCommand extends CommandBase {

    private Lift lift;
    private double target = 0;

    boolean f = true;

    public LiftCommand(Lift lift, double targetPosition) {
        this.lift = lift;
        this.target = targetPosition;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (f) {
            if (target > Lift.l2 && target < Lift.l1) {
                lift.setLimit(true);
            }
            lift.setTarget(target);
            f = false;
        }
    }

    @Override
    public boolean isFinished() {
        return lift.check();
    }

}
