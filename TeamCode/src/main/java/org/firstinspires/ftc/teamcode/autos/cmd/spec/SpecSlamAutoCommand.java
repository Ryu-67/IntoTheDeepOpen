package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class SpecSlamAutoCommand extends SequentialCommandGroup {

    public SpecSlamAutoCommand(Deposit deposit, Lift lift, MPPivot pivot)
    {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new WaitCommand(50),
                new LiftCommand(lift, 80),
                new ArmCommand(deposit, ArmCommand.DepositState.floorSpec, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
        );
    }

}
