package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftLimitCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class FlipAndExtendCommand extends SequentialCommandGroup {

    public FlipAndExtendCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new LiftLimitCommand(lift, true),
                new LiftCommand(lift, 0),
                new PivotCommand(pivot, 90),
                new ParallelCommandGroup(
                        new LiftCommand(lift, 2280)),
                        new ParallelCommandGroup(
                                new WaitCommand(300),
                                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                        )
        );
    }

}
