package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class PickoutCommand extends SequentialCommandGroup {

    public PickoutCommand(Deposit deposit, Lift lift) {
        addCommands(
                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new LiftCommand(lift, 0)
        );
    }

}
