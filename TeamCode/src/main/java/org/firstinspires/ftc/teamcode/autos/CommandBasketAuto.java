package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.RoadrunnerCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.DropAndResetAutoCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.DropAndResetCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.FlipAndExtendAutoCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.FlipAndExtendCommand;
import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class CommandBasketAuto extends LinearOpMode {

    public SparkFunOTOSDrive drive; public Lift lift; public MPPivot pivot; public Deposit deposit;

    public Robot robot;

    public TrajectoryActionBuilder initialDeposit, n1, n1Deposit, n2, n2Deposit, n3, n3Deposit, park;
    public Action id, n1t, n1dt, n2t, n2dt, n3t, n3dt, pt;

    public ElapsedTime loopTimer = new ElapsedTime();
    public double i = 0, i1 = 0;

    CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, false, false, false);
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-40.5, -64.5, Math.toRadians(90)));
        lift = robot.lift;
        pivot = robot.pivot;
        deposit = robot.deposit;

        initialDeposit = drive.actionBuilder(new Pose2d(-40.5, -64.5, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(225));

        n1 = drive.actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48.5, -41, Math.toRadians(90)), Math.PI/2);

        n1Deposit = drive.actionBuilder(new Pose2d(-48.5, -41, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(225));

        n2 = drive.actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(90)), Math.PI/2);

        n2Deposit = drive.actionBuilder(new Pose2d(-58, -41, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(225));

        n3 = drive.actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-58.65, -36.25, Math.toRadians(140)), Math.PI);

        n3Deposit = drive.actionBuilder(new Pose2d(-58.65, -35.85, Math.toRadians(140)))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(180));

        park = drive.actionBuilder(new Pose2d(-59, -59, Math.toRadians(45)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-33, -13, Math.toRadians(0)), 0);

        id = initialDeposit.build();
        n1t = n1.build();
        n1dt = n1Deposit.build();
        n2t = n2.build();
        n2dt = n2Deposit.build();
        n3t = n3.build();
        n3dt = n3Deposit.build();
        pt = park.build();

        actionQueue.schedule(
                //Initial Deposit Command
                new ParallelCommandGroup(
                        new RoadrunnerCommand(id),
                        new SequentialCommandGroup(
                                new LiftCommand(lift, 2620),
                                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                        )
                ),
                //Drop + go to intake
                new SequentialCommandGroup(
                        new DropAndResetAutoCommand(deposit, lift, pivot),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(n1t),
                                new PivotCommand(pivot, 0)
                        )
                ),
                //Intake and back to Basket
                new SequentialCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new FlipAndExtendAutoCommand(deposit, lift, pivot),
                                new RoadrunnerCommand(n1dt)
                        )
                ),
                //Drop + go to intake
                new SequentialCommandGroup(
                        new DropAndResetAutoCommand(deposit, lift, pivot),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(n1t),
                                new PivotCommand(pivot, 0)
                        )
                ),
                //Intake and back to Basket
                new SequentialCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new FlipAndExtendAutoCommand(deposit, lift, pivot),
                                new RoadrunnerCommand(n2dt)
                        )
                ),
                //Drop + go to intake
                new SequentialCommandGroup(
                        new DropAndResetAutoCommand(deposit, lift, pivot),
                        new ParallelCommandGroup(
                                new RoadrunnerCommand(n1t),
                                new PivotCommand(pivot, 0)
                        )
                ),
                //Intake and back to Basket
                new SequentialCommandGroup(
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new FlipAndExtendAutoCommand(deposit, lift, pivot),
                                new RoadrunnerCommand(n3dt)
                        )
                ),
                //park
                new SequentialCommandGroup(
                        new DropAndResetAutoCommand(deposit, lift, pivot),
                        new RoadrunnerCommand(pt)
                )
        );

        pivot.setTargetAngle(MainArm.vertAngle);

        while (!opModeIsActive() && opModeInInit()) {
            pivot.update();
            drive.otos.setPosition(new SparkFunOTOS.Pose2D(-40.5, -64.5, Math.toRadians(90)));
        }

        while (opModeIsActive()) {
            robot.update();
            actionQueue.run();

            i1 = loopTimer.milliseconds();
            telemetry.addData("Looptime", 1000/(i1-i));
            telemetry.update();
            i = i1;

            if (isStopRequested()) {
                actionQueue.reset();
                stop();
            }
        }
    }
}