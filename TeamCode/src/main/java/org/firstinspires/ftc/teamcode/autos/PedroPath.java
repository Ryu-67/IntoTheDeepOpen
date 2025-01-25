package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ConfigArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PedroCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.PUTTHESPECDOWNAutoCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.PickAndFlipCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Pedro Pathing FREAKY Spec Auto")
@Config
public class PedroPath extends LinearOpMode {

    Follower pedroFollower; Deposit deposit; Lift lift; MPPivot pivot;
    Robot robot;

    public static double lastPushX = 16;
    public static double tValLastPush = 0.9825;

    PathChain preload, push, firstCycle, back, score, back2;

    CommandScheduler s = CommandScheduler.getInstance();

    //arm for wall grab" 0.45 a, 0.8p, 100 l, 90 p

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, false, true, false);
        lift = robot.lift; deposit = robot.deposit; pivot = robot.pivot;
        pedroFollower = new Follower(hardwareMap);

        preload =  new PathBuilder().addPath((new BezierLine(
                new Point(7.500, 64.5, Point.CARTESIAN),
                new Point(37 , 74.529, Point.CARTESIAN)
        ))).setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(4)
                .setPathEndTimeoutConstraint(0.1).build();

        push = new PathBuilder().addPath(
                new BezierCurve(
                        new Point(37, 74.529, Point.CARTESIAN),
                        new Point(26.701, 33.492, Point.CARTESIAN),
                        new Point(28.244, 30, Point.CARTESIAN),
                        new Point(50, 34.881, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(58, 34.881, Point.CARTESIAN),
                                new Point(58, 23.151, Point.CARTESIAN),
                                new Point(26, 23.151, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(26, 23.151, Point.CARTESIAN),
                                new Point(58, 26, Point.CARTESIAN),
                                new Point(62, 13.273, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(58, 13.273, Point.CARTESIAN),
                                new Point(26, 13.119, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(26, 13.119, Point.CARTESIAN),
                                new Point(54, 13.273, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(54, 13.273, Point.CARTESIAN),
                                new Point(61.428, 8, Point.CARTESIAN),
                                new Point(24, 11, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(24, 11.000, Point.CARTESIAN),
                                new Point(16.661, 35.107, Point.CARTESIAN),
                                new Point(8.1, 33.471, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(0)
                .setPathEndTranslationalConstraint(0.25)
                .setPathEndTimeoutConstraint(0.95)
                .build();

        firstCycle = new PathBuilder().addPath(
                new BezierCurve(
                        new Point(8.1, 33.471, Point.CARTESIAN),
                        new Point(15.025, 72.595, Point.CARTESIAN),
                        new Point(37, 70.810, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.95).build();

        back = new PathBuilder().addPath(
                new BezierLine(
                        new Point(37, 70.810, Point.CARTESIAN),
                        new Point(8, 37, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.95).build();

        score = new PathBuilder().addPath(
                new BezierCurve(
                        new Point(8, 37, Point.CARTESIAN),
                        new Point(14.281, 66.050, Point.CARTESIAN),
                        new Point(37, 68, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.95).build();

        back2 = new PathBuilder().addPath(
                new BezierCurve(
                        new Point(37, 68, Point.CARTESIAN),
                        new Point(23, 65.157, Point.CARTESIAN),
                        new Point(8, 37, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTranslationalConstraint(2)
                .setPathEndTimeoutConstraint(0.95).build();

        s.reset();
        s = CommandScheduler.getInstance();

        pivot.setTargetAngle(90);

        s.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new PedroCommand(pedroFollower, preload)
                                ),
                                new LiftCommand(lift, 700),
                                new PivotCommand(pivot, 50)
                        ),

                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                        new SequentialCommandGroup(
                                                new ParallelRaceGroup(
                                                        new LiftCommand(lift, 0),
                                                        new WaitCommand(300)
                                                ),
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new PivotCommand(pivot, 90),
                                                                new LiftCommand(lift, 70),
                                                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped)
                                                        ),
                                                        new PedroCommand(pedroFollower, push)
                                                )
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                                new PedroCommand(pedroFollower, firstCycle),
                                                new LiftCommand(lift, 700),
                                                new PivotCommand(pivot, 50)
                                        )
                                )

                        ),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                                new SequentialCommandGroup(
                                                        new LiftCommand(lift, 0),
                                                        new PivotCommand(pivot, 90),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                                                                new LiftCommand(lift, 70)
                                                        )
                                                ),
                                                new PedroCommand(pedroFollower, back)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                                new PedroCommand(pedroFollower, score),
                                                new LiftCommand(lift, 720),
                                                new PivotCommand(pivot, 50)
                                        )
                                )

                        ),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                                new SequentialCommandGroup(
                                                        new LiftCommand(lift, 0),
                                                        new PivotCommand(pivot, 90),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                                                                new LiftCommand(lift, 70)
                                                        )
                                                ),
                                                new PedroCommand(pedroFollower, back2)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                                new PedroCommand(pedroFollower, score),
                                                new LiftCommand(lift, 750),
                                                new PivotCommand(pivot, 50)
                                        )
                                )

                        ),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                                new SequentialCommandGroup(
                                                        new LiftCommand(lift, 0),
                                                        new PivotCommand(pivot, 90),
                                                        new ParallelCommandGroup(
                                                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.open, ArmCommand.WristState.wrapped),
                                                                new LiftCommand(lift, 70)
                                                        )
                                                ),
                                                new PedroCommand(pedroFollower, back2)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new ArmCommand(deposit, ArmCommand.DepositState.wallSpecUp, ArmCommand.ClawState.closed, ArmCommand.WristState.wrapped),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                                                new PedroCommand(pedroFollower, score),
                                                new LiftCommand(lift, 760),
                                                new PivotCommand(pivot, 50)
                                        )
                                )

                        ),
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.dropSlam, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                                new SequentialCommandGroup(
                                                        new LiftCommand(lift, 0),
                                                        new PivotCommand(pivot, 0)
                                                ),
                                                new PedroCommand(pedroFollower,
                                                        new PathBuilder().addPath(new BezierLine(new Point(37, 68), new Point(15, 33)))
                                                                .setLinearHeadingInterpolation(0, Math.toRadians(0)).build())
                                        )
                                )
                        )





//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                            new SequentialCommandGroup(
//                                    new WaitCommand(120),
//                                    new ParallelRaceGroup(
//                                           new LiftCommand(lift, 0),
//                                           new WaitCommand(300)
//                                    )
//                            )
//                        ),
//                        new ParallelCommandGroup(
//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new PivotCommand(pivot, 0),
//                                                new SequentialCommandGroup(
//                                                        new WaitCommand(500),
//                                                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
//                                                ),
//                                                new PedroCommand(pedroFollower, back)
//                                        )
//                                )
//                        ),
//                        new SequentialCommandGroup(
//                            new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
//                            new ParallelCommandGroup(
//                                   new SequentialCommandGroup(
//                                            new PivotCommand(pivot, 90),
//                                            new ConfigArmCommand(
//                                                    deposit, ConfigArmCommand.DepositState.specDepo, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                            )
//                                    ),
//                                   new PedroCommand(pedroFollower, score),
//                                    new LiftCommand(lift, PickAndFlipCommand.specHeight)
//                            )
//                        ),
//                        new ParallelCommandGroup(
//                            new ConfigArmCommand(
//                                    deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                            ),
//                            new SequentialCommandGroup(
//                                    new WaitCommand(120),
//                                    new ParallelRaceGroup(
//                                            new LiftCommand(lift, 0),
//                                            new WaitCommand(300)
//                                   )
//                            )
//                        ),
//                            new ParallelCommandGroup(
//                                    new ConfigArmCommand(
//                                            deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new PivotCommand(pivot, 0),
//                                                new SequentialCommandGroup(
//                                                        new WaitCommand(500),
//                                                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
//                                                ),
//                                                new PedroCommand(pedroFollower, back2)
//                                        )
//                                )
//                            ),
//                        new SequentialCommandGroup(
//                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new PivotCommand(pivot, 90),
//                                                new ConfigArmCommand(
//                                                        deposit, ConfigArmCommand.DepositState.specDepo, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                                )
//                                        ),
//                                        new PedroCommand(pedroFollower, score),
//                                        new LiftCommand(lift, PickAndFlipCommand.specHeight)
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(120),
//                                        new ParallelRaceGroup(
//                                                new LiftCommand(lift, 0),
//                                                new WaitCommand(300)
//                                        )
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new PivotCommand(pivot, 0),
//                                                new SequentialCommandGroup(
//                                                        new WaitCommand(500),
//                                                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
//                                                ),
//                                                new PedroCommand(pedroFollower, back2)
//                                        )
//                                )
//                        ),
//                        new SequentialCommandGroup(
//                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new PivotCommand(pivot, 90),
//                                                new ConfigArmCommand(
//                                                        deposit, ConfigArmCommand.DepositState.specDepo, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                                )
//                                        ),
//                                        new PedroCommand(pedroFollower, score),
//                                        new LiftCommand(lift, PickAndFlipCommand.specHeight)
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(120),
//                                        new ParallelRaceGroup(
//                                                new LiftCommand(lift, 0),
//                                                new WaitCommand(300)
//                                        )
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new ConfigArmCommand(
//                                        deposit, ConfigArmCommand.DepositState.specSlam, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.wrapped, 0.2
//                                ),
//                                new SequentialCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new PivotCommand(pivot, 0),
//                                                new SequentialCommandGroup(
//                                                        new WaitCommand(500),
//                                                        new ArmCommand(deposit, ArmCommand.DepositState.specIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal)
//                                                ),
//                                                new PedroCommand(pedroFollower, back2)
//                                        )
//                                )
//                        )
//
//                        new SequentialCommandGroup(
//                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.2),
//                                new ParallelCommandGroup(
//                                        new PivotCommand(pivot, 90),
//                                        new PedroCommand(pedroFollower, score),
//                                        new LiftCommand(lift, 200)
//                                )
//                        ),
//
//                        new PUTTHESPECDOWNAutoCommand(deposit, lift, pivot, pedroFollower, back2),
//
//                        new SequentialCommandGroup(
//                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.2),
//                                new ParallelCommandGroup(
//                                        new PivotCommand(pivot, 90),
//                                        new PedroCommand(pedroFollower, score),
//                                        new LiftCommand(lift, 200)
//                                )
//                        ),
//
//                        new PUTTHESPECDOWNAutoCommand(deposit, lift, pivot, pedroFollower, back2),
//
//                        new SequentialCommandGroup(
//                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.specIntake, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.2),
//                                new ParallelCommandGroup(
//                                        new PivotCommand(pivot, 90),
//                                        new PedroCommand(pedroFollower, score),
//                                        new LiftCommand(lift, 200)
//                                )
//                        )
                )
        );

        while (opModeInInit()) {
            pedroFollower.setPose(new Pose(7.500, 64.5, Math.toRadians(0)));
            pivot.update();
        }


        while (opModeIsActive()) {
            s.run();
            pedroFollower.update();
            lift.update();
            pivot.update();
        }
    }

}
