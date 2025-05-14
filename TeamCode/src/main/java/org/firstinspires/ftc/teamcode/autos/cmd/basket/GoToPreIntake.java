package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class GoToPreIntake extends SequentialCommandGroup {
    public GoToPreIntake(Deposit deposit, MPPivot pivot, Lift lift) {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.preIntakeLifted, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new PivotCommand(pivot, 0),
                new LiftCommand(lift, Lift.l2)
        );
    }
}
