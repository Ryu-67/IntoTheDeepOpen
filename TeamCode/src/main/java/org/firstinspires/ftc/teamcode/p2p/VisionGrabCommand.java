package org.firstinspires.ftc.teamcode.p2p;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class VisionGrabCommand extends SequentialCommandGroup {


    public VisionGrabCommand(PointRunner runner, CameraSensor sensor, MPPivot pivot, Lift lift, Deposit deposit) {
        addCommands(
                new CamDrive.DetectSampleCommand(sensor),
                new ParallelCommandGroup(
                        new VisionDTAlignCommand(runner, sensor),
                        new SequentialCommandGroup(
                                new PivotCommand(pivot, 0),
                                new LiftCommand(lift, 0)
                        )
                ),
                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
        );
    }

}
