package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class PickAndFlipCommand extends SequentialCommandGroup {

    public static double specHeight = 910;

    public PickAndFlipCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new LiftLimitCommand(lift, false),
                new LiftCommand(lift, 0),
                new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.specDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new PivotCommand(pivot,  90)
                ),
                new LiftLimitCommand(lift, true),
                new LiftCommand(lift, specHeight)
        );
    }

}