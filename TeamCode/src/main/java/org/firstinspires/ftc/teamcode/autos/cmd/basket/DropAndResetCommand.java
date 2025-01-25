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

public class DropAndResetCommand extends SequentialCommandGroup {

    public DropAndResetCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                        ),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
                                )
                        )
                ),
                new LiftLimitCommand(lift, false)
        );
    }

}
