package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ConfigArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class Grasp extends SequentialCommandGroup {
    public Grasp(Deposit deposit, Lift lift) {
        addCommands(
                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.current, 0.15),
                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.current),
                new ParallelCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new LiftCommand(lift, 0)
                        )
                )
        );
    }
}
