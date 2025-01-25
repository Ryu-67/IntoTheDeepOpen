package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class ToSlamCommand extends SequentialCommandGroup {

    public ToSlamCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 500),
                                new PivotCommand(pivot, 50)
                        )
                )
        );
    }

}
