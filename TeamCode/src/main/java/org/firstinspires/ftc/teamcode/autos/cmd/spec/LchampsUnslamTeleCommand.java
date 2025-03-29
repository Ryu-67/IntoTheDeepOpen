package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class LchampsUnslamTeleCommand extends ParallelCommandGroup {

    public LchampsUnslamTeleCommand(Lift lift, MPPivot pivot, Deposit deposit) {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new SequentialCommandGroup(
                        new LiftCommand(lift, 0),
                        new PivotCommand(pivot, 94),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                                new LiftCommand(lift, 160)
                        )
                )
        );
    }
}
