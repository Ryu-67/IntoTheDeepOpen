package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class DropAndResetAutoCommand extends SequentialCommandGroup {

    public DropAndResetAutoCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new LiftCommand(lift, 0)
        );
    }

}
