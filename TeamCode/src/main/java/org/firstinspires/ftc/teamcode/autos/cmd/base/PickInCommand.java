package org.firstinspires.ftc.teamcode.autos.cmd.base;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class PickInCommand extends SequentialCommandGroup {

    public PickInCommand(Deposit deposit, Lift lift) {
        addCommands(
                new LiftCommand(lift, 1250),
                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
        );
    }

}
