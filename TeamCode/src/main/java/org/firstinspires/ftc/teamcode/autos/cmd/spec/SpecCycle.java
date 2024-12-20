package org.firstinspires.ftc.teamcode.autos.cmd.spec;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.RoadrunnerCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class SpecCycle extends SequentialCommandGroup {

    public SpecCycle(Lift lift, MPPivot pivot, Deposit deposit, Action pickupDriveAction, Action depositDriveAction) {
        addCommands(
                new SpecSlamAutoCommand(deposit, lift, pivot),
                new ParallelCommandGroup(
                        new RoadrunnerCommand(pickupDriveAction),
                        new PivotCommand(pivot, 0)
                ),
                new ArmCommand(deposit, ArmCommand.DepositState.floorSpec, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new ParallelCommandGroup(
                        new FloorPickAndFlipCommand(deposit, lift, pivot),
                        new RoadrunnerCommand(depositDriveAction)
                )
        );
    }

}
