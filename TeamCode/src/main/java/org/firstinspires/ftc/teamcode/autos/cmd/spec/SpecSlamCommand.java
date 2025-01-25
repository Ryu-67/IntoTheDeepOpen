package org.firstinspires.ftc.teamcode.autos.cmd.spec;

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

public class SpecSlamCommand extends SequentialCommandGroup {

    public SpecSlamCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new LiftLimitCommand(lift, true),
                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new ParallelCommandGroup(
                        new LiftCommand(lift, 0),
                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                        new PivotCommand(pivot, 0)
                ),
                new LiftLimitCommand(lift, false)
        );
    }

}
