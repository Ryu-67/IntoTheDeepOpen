package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCompletionCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.RoadrunnerCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.PickAndFlipCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.SpecCycle;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.SpecSlamAutoCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.SpecSlamCommand;
import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "freaktonomous")
public class FreakySpecAuto extends LinearOpMode {

    public Robot robot;

    public Lift lift; public MPPivot pivot; public Deposit deposit; public SparkFunOTOSDrive drive;

    Action initial, grab1, drop1, grab2, drop2, grab3, drop3, precycle, slam1, pick, cycle, pick2, slam3;

    CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, false, true, false);
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(7.4, -63.5, Math.toRadians(-90)));
        lift = robot.lift; pivot = robot.pivot; deposit = robot.deposit;

        initial = drive.actionBuilder(new Pose2d(7.4, -63.5, Math.toRadians(-90)))
                .setTangent(-Math.toRadians(90))
                .lineToY(-35)
                .build();

        grab1 = drive.actionBuilder(new Pose2d(7.4, -35, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(26, -38, Math.toRadians(26)), Math.toRadians(26))
                .build();

        drop1 = drive.actionBuilder(new Pose2d(26, -38, Math.toRadians(26)))
                .turn(Math.toRadians(-65))
                .build();

        grab2 = drive.actionBuilder(new Pose2d(26, -38, Math.toRadians(26-65)))
                .setTangent(0)
                .lineToXSplineHeading(34, Math.toRadians(24))
                .build();

        drop2 = drive.actionBuilder(new Pose2d(34, -38, Math.toRadians(24)))
                .turn(-Math.toRadians(65))
                .build();

        grab3 = drive.actionBuilder(new Pose2d(34, -38, Math.toRadians(24-65)))
                .setTangent(0)
                .lineToXSplineHeading(43, Math.toRadians(22))
                .build();

        drop3 = drive.actionBuilder(new Pose2d(26, -38, Math.toRadians(24-65)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(42, -50, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        precycle = drive.actionBuilder(new Pose2d(42, -50, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .lineToY(-55)
                .build();

        slam1 = drive.actionBuilder(new Pose2d(36, -52, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, -35), Math.toRadians(90))
                .build();

        pick = drive.actionBuilder(new Pose2d(10, -35, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(37, -50, Math.toRadians(-45)), Math.toRadians(-45))
                .build();

        cycle = drive.actionBuilder(new Pose2d(37, -50, Math.toRadians(-45)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        pivot.setTargetAngle(MainArm.vertAngle);
        deposit.setDeposit(0, 0.01, 0.01, Claw.closed);

        while (!opModeIsActive() && opModeInInit()) {
            pivot.update();
            drive.otos.setPosition(new SparkFunOTOS.Pose2D(7.4, -63.5, Math.toRadians(-90)));
        }

        actionQueue.schedule(
                //Initial slam
                new ParallelCommandGroup(
                        new RoadrunnerCommand(initial),
                        new LiftCommand(lift, PickAndFlipCommand.specHeight),
                        new ArmCommand(deposit, ArmCommand.DepositState.specDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                ),
                //Grabcycles
                new SequentialCommandGroup(
                        new SpecSlamAutoCommand(deposit, lift, pivot),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.vertical),
                                new SequentialCommandGroup(
                                        new PivotCompletionCommand(pivot, 0),
                                        new LiftCommand(lift, Lift.l2)
                                ),
                                new RoadrunnerCommand(grab1)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.vertical),
                        new RoadrunnerCommand(drop1),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.vertical),
                        new RoadrunnerCommand(grab2),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.vertical),
                        new RoadrunnerCommand(drop2),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.vertical),
                        new RoadrunnerCommand(grab3),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.vertical),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(drop3),
                                new LiftCommand(lift, 0)
                        )
                ),
                //Final drop
                new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                new RoadrunnerCommand(precycle),
                //Begin Cycles
                new ParallelCommandGroup(
                        new PickAndFlipCommand(deposit, lift, pivot),
                        new RoadrunnerCommand(slam1)
                ),
                new SpecCycle(lift, pivot, deposit, pick, cycle),
                new SpecCycle(lift, pivot, deposit, pick, cycle),
                new SpecCycle(lift, pivot, deposit, pick, cycle)
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
