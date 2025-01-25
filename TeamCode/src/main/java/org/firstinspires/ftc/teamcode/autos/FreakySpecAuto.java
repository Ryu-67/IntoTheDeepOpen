package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.RoadrunnerCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.PreSlamCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.ToSlamCommand;
import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDriveRR;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "freaktonomous")
public class FreakySpecAuto extends LinearOpMode {

    public Robot robot;

    public Lift lift; public MPPivot pivot; public Deposit deposit; public SparkFunOTOSDriveRR drive;

    Action initial, setup, d1, t1, d2, t2, d3, initialCycle, cycle, cycle2, returnC, return2, precycle, MOVE;

    CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, false, true, false);
        drive = new SparkFunOTOSDriveRR(hardwareMap, new Pose2d(7.4, -63.5, Math.toRadians(90)));
        lift = robot.lift; pivot = robot.pivot; deposit = robot.deposit;


        initial = drive.actionBuilder(new Pose2d(7.4, -63.5, Math.toRadians(90)))
                .setTangent(-Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-4, -29), Math.toRadians(90))
                .build();

        setup = drive.actionBuilder(new Pose2d(-4, -29, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .lineToY(-42)
                .setTangent(0)
                .lineToXLinearHeading(46, Math.toRadians(90))
//                .setTangent(Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(31.8, -33, Math.toRadians(45)), Math.toRadians(45))
                .build();

        d1 = drive.actionBuilder(new Pose2d(46, -42, Math.toRadians(90)))
                .setTangent(-90)
                .lineToY(-52)
//                .turnTo(Math.toRadians(-45))
                .build();

        t1 = drive.actionBuilder(new Pose2d(46, -52, Math.toRadians(90)))
//                        .setTangent(0)
//                .turnTo(Math.toRadians(24.2))
//                .lineToX(32)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56.5, -42), Math.toRadians(90))
                                .build();

        d2 = drive.actionBuilder(new Pose2d(56.5, -42, Math.toRadians(90)))
                         .setTangent(Math.toRadians(-90))
                .lineToY(-52)
                                .build();

        t2 = drive.actionBuilder(new Pose2d(56.5, -52, Math.toRadians(90)))
//                        .setTangent(0)
//                                .lineToXSplineHeading(41, Math.toRadians(24))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(58, -40, Math.toRadians(50)), Math.toRadians(50))
                                        .build();

        d3 = drive.actionBuilder(new Pose2d(58, -40, Math.toRadians(50)))
//                        .turnTo(Math.toRadians(-45))
                .setTangent(Math.toRadians(220))
                .splineToLinearHeading(new Pose2d(50, -52, Math.toRadians(90)), Math.toRadians(-90))
                                .build();

        precycle = drive.actionBuilder(new Pose2d(57, -44, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, -62.5), Math.toRadians(-90))
                                        .build();

        initialCycle = drive.actionBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-3, -29, Math.toRadians(90)), Math.toRadians(90))
                .build();

        returnC = drive.actionBuilder(new Pose2d(-3, -29, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(37, -62, Math.toRadians(90)), Math.toRadians(-45))
                .build();

        cycle = drive.actionBuilder(new Pose2d(37, -62, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-2, -27, Math.toRadians(90)), Math.toRadians(90))
                .build();

        return2 = drive.actionBuilder(new Pose2d(-2, -27, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(37, -63, Math.toRadians(90)), Math.toRadians(-45))
                        .build();

        cycle2 = drive.actionBuilder(new Pose2d(37, -63, Math.toRadians(90)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-4, -27, Math.toRadians(90)), Math.toRadians(90))
                .build();

        MOVE = drive.actionBuilder(new Pose2d(-4, -27, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .lineToY(-50)
                .build();

        pivot.setTargetAngle(MainArm.vertAngle);

        deposit.setDeposit(Deposit.hori, 0.01, 0.01, Claw.closed);

        actionQueue.reset();
        actionQueue = CommandScheduler.getInstance();
        actionQueue.cancelAll();
        actionQueue.disable();
        actionQueue.enable();

        while (!opModeIsActive() && opModeInInit()) {
            pivot.update();
            drive.otos.setPosition(new SparkFunOTOS.Pose2D(7.4, -63.5, Math.toRadians(90)));
        }

        actionQueue.schedule(
                new SequentialCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(initial),
                                new ToSlamCommand(deposit, lift, pivot)
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                new RoadrunnerCommand(setup),
                                new SequentialCommandGroup(
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 0),
                                        new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                        new WaitCommand(1000),
                                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new RoadrunnerCommand(d1),
                                                new PivotCommand(pivot, 90)
                                        ),
                                        new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
                                )
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                new PivotCommand(pivot, 0),
                                new LiftCommand(lift, 0),
                                new RoadrunnerCommand(t1)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake   , ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.closed, ArmCommand.WristState.vertical),
                                new PivotCommand(pivot, 90),
                                new RoadrunnerCommand(d2)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, 0),
                                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.diagonal),
                                new RoadrunnerCommand(t2)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake   , ArmCommand.ClawState.closed, ArmCommand.WristState.diagonal),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                new PivotCommand(pivot, 90),
                                new LiftCommand(lift, 0),
                                new RoadrunnerCommand(d3)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.shoot, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                        new SequentialCommandGroup(
                                new RoadrunnerCommand(precycle),
                               new PreSlamCommand(pivot, lift, deposit)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                        new ToSlamCommand(deposit, lift, pivot),
                                        new RoadrunnerCommand(initialCycle)
                                )
                        ),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(returnC),
                                new SequentialCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                        new LiftCommand(lift, 0),
                                        new PreSlamCommand(pivot, lift, deposit)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                        new ToSlamCommand(deposit, lift, pivot),
                                        new RoadrunnerCommand(cycle)
                                )
                        ),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(return2),
                                new SequentialCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                        new LiftCommand(lift, 0),
                                        new PreSlamCommand(pivot, lift, deposit)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                        new ToSlamCommand(deposit, lift, pivot),
                                        new RoadrunnerCommand(cycle2)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new RoadrunnerCommand(MOVE),
                                                new ArmCommand(deposit, ArmCommand.DepositState.specSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
                                        ),
                                        new PivotCommand(pivot, 0)
                                ),
                                new LiftCommand(lift, 0)
                        )
                )
        );

        while (opModeIsActive()) {
            robot.update();

            actionQueue.run();

            if (isStopRequested()) {
                actionQueue.reset();
                stop();
            }
        }

    }
}
