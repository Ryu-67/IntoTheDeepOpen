package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class EnterSpecState extends ParallelCommandGroup {
    public EnterSpecState(Lift lift, Deposit deposit, MPPivot pivot) {
        addCommands(
                new LiftCommand(lift,0),
                new PivotCommand(pivot, 110),
                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped)
        );
    }
}
