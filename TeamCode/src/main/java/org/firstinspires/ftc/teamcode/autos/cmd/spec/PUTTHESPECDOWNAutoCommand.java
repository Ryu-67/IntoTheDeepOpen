package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ConfigArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PedroCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class PUTTHESPECDOWNAutoCommand extends SequentialCommandGroup {

    public PUTTHESPECDOWNAutoCommand(Deposit deposit, Lift lift, MPPivot pivot, Follower follower, PathChain pathChain) {
        addCommands(
                new SequentialCommandGroup(
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
                                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                        new PedroCommand(follower, pathChain)
                                )
                        )
                )
        );
    }

}
