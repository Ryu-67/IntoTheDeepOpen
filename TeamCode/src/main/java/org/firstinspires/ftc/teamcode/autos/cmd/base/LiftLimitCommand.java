package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftLimitCommand extends CommandBase {

    private Lift lift;

    private boolean set = false;

    public LiftLimitCommand(Lift lift, boolean vert) {
        this.lift = lift;
        this.set = vert;
    }


    @Override
    public boolean isFinished() {
        lift.setLimit(set);
        return true;
    }

}
