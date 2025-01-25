package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class FloorPickAndFlipCommand extends SequentialCommandGroup {

    public static double specHeight = 130;

    public FloorPickAndFlipCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new ParallelCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                        new PivotCommand(pivot,  90)
                ),
                new LiftCommand(lift, specHeight)
        );
    }

}
