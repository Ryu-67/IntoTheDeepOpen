package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PedroCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class LchampsSlamTeleCommand extends ParallelCommandGroup {

    public LchampsSlamTeleCommand(Lift lift, MPPivot pivot, Deposit deposit) {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                new LiftCommand(lift, 710),
                                new PivotCommand(pivot, 50)
                        )
                )
        );
    }

}
