package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class UnSlamTeleCommand extends SequentialCommandGroup {

    public UnSlamTeleCommand(MPPivot pivot, Lift lift, Deposit deposit) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                new LiftCommand(lift, 0)
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftCommand(lift, 200),
                                        new PivotCommand(pivot, 90)
                                ),
                                new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped)
                        )
                )
        );
    }

}
