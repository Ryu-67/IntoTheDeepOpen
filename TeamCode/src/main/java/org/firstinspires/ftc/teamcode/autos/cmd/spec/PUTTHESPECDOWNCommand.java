package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ConfigArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class PUTTHESPECDOWNCommand extends SequentialCommandGroup {

    public PUTTHESPECDOWNCommand(Deposit deposit, Lift lift, MPPivot pivot) {
        addCommands(
                new ParallelCommandGroup(
                        new ConfigArmCommand(
                                deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ParallelRaceGroup(
                                    new LiftCommand(lift, 0),
                                        new WaitCommand(300)
                                )
                        )
                ),
                        new ConfigArmCommand(
                                deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.wrapped, 0.2
                        ),
                new ParallelCommandGroup(
                        new PivotCommand(pivot, 0),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
                        )
                )
        );
    }

}
