package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class FlipAndExtendAutoCommand extends SequentialCommandGroup {

    public FlipAndExtendAutoCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new LiftCommand(lift, 0),
                new PivotCommand(pivot, 90),
                new LiftCommand(lift, 2620),
                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
        );
    }

}
