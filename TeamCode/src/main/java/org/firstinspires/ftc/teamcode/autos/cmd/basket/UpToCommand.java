package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftLimitCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCompletionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class UpToCommand extends SequentialCommandGroup {

    public UpToCommand(Lift lift, MPPivot pivot) {
        addCommands(
                new PivotCompletionCommand(pivot, 90),
                new LiftLimitCommand(lift, true),
                new LiftCommand(lift, Lift.l1)
        );
    }

}
