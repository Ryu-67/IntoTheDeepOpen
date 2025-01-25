package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class GoToSlamTeleCommand extends SequentialCommandGroup {

    public GoToSlamTeleCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new SequentialCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 500),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 50),
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                                )
                        )
                )
        );
    }

}
