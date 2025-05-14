package org.firstinspires.ftc.teamcode.autos;

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
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCompletionCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftLimitCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PedroCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCompletionCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Freaky 4 Basket Auto")
public class PedroBasketAutoFREAKEdition extends LinearOpMode {

    public Robot robot;
    public Lift lift; public MPPivot pivot; public Deposit deposit;

    public Follower follower;

    public PathChain preload, pick1, drop1, pick2, drop2, pick3, drop3, park;

    CommandScheduler s;

    @Override
    public void runOpMode() throws InterruptedException {
        FollowerConstants.xMovement = 30;
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
                0.018,
                0,
                0.000001,
                0.6,
                0
        );
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(
                0.1,
                0,
                0,
                0
        );

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(
                0.8,
                0,
                0,
                0
        );

        robot = new Robot(hardwareMap, false, false, false);

        lift = robot.lift; pivot = robot.pivot; deposit = robot.deposit;

        follower = new Follower(hardwareMap);

        preload = new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.50, 113.000, Point.CARTESIAN),
                                new Point(20, 120, Point.CARTESIAN),
                                new Point(16, 130, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndVelocityConstraint(0)
                .build();

        pick1 = new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(16, 130, Point.CARTESIAN),
                                new Point(25, 125, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTimeoutConstraint(400)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(0))
                .setPathEndVelocityConstraint(0)
                .build();

        drop1 = new PathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(25, 125, Point.CARTESIAN),
                                new Point(16, 130, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTimeoutConstraint(400)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndVelocityConstraint(0)
                .build();

        pick2 = new PathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(16, 130, Point.CARTESIAN),
                                new Point(25.7, 131, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setPathEndTimeoutConstraint(600)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(3)).setPathEndVelocityConstraint(0)
                .build();

        drop2 = new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(25.7, 131, Point.CARTESIAN),
                                new Point(16, 130, Point.CARTESIAN)
                        )
                )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setPathEndTimeoutConstraint(0.95)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndVelocityConstraint(0)
                .build();

        pick3 = new PathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(16, 130, Point.CARTESIAN),
                                new Point(26, 130, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(47.5))
                .setPathEndTimeoutConstraint(0.95)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndVelocityConstraint(0)
                .build();

        drop3 = new PathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(26, 130, Point.CARTESIAN),
                                new Point(16, 130, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(47.5), Math.toRadians(-50))
                .setPathEndTimeoutConstraint(0.95)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndVelocityConstraint(0)
                .build();

        park = new PathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(16, 130, Point.CARTESIAN),
                                new Point(61.438, 120, Point.CARTESIAN),
                                new Point(62.777, 99.074, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-90))
                .setPathEndTimeoutConstraint(0.95)
                .setPathEndTranslationalConstraint(0.2)
                .setPathEndHeadingConstraint(Math.toRadians(1))
                .setPathEndVelocityConstraint(0)
                .build();

        s = CommandScheduler.getInstance();
        s.reset();
        s = CommandScheduler.getInstance();

        pivot.setTargetAngle(90);


        while (opModeInInit()) {
            follower.setPose(new Pose(7.500, 113, Math.toRadians(0)));
            follower.setStartingPose(new Pose(7.500, 113, Math.toRadians(0)));
            pivot.update();
        }

        s.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new PedroCommand(follower, preload),
                                new SequentialCommandGroup(
                                        new LiftLimitCommand(lift, true),
                                        new ParallelRaceGroup(
                                                new LiftCommand(lift, 870),
                                                new WaitCommand(800)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.6),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.3),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, pick1),
                                new SequentialCommandGroup(
                                        new LiftCompletionCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, drop1),
                                new SequentialCommandGroup(
                                        new PivotCompletionCommand(pivot, 90),
                                        new LiftLimitCommand(lift, true),
                                        new ParallelRaceGroup(
                                                new LiftCommand(lift, 870),
                                                new WaitCommand(800)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, pick2),
                                new SequentialCommandGroup(
                                        new LiftCompletionCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, drop2),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 90),
                                        new LiftLimitCommand(lift, true),
                                        new ParallelRaceGroup(
                                                new LiftCommand(lift, 860),
                                                new WaitCommand(800)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.diagonal2, 0.2),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new LiftCompletionCommand(lift, 0),
                                        new WaitCommand(300)
                                ),
                                new ParallelRaceGroup(
                                        new PivotCompletionCommand(pivot, 0),
                                        new WaitCommand(200)
                                ),
                                new PedroCommand(follower, pick3)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.diagonal2),
                        new ParallelCommandGroup(
                                new PedroCommand(follower, drop3),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 90),
                                        new LiftLimitCommand(lift, true),
                                        new ParallelRaceGroup(
                                                new LiftCommand(lift, 860),
                                                new WaitCommand(800)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.3),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.antiSlamDunkTechnology, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, 0.2),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftCommand(lift, 0),
                                        new ParallelCommandGroup(
                                                new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.open, ArmCommand.WristState.horizontal),
                                                new PivotCommand(pivot, 30)
                                        )
                                ),
                                new PedroCommand(follower, park)
                        )
                )
        );

        while (opModeIsActive()) {
            s.run();
            follower.update();
            lift.update();
            pivot.update();
            telemetry.addData("Liftpos", lift.ticks);
            telemetry.update();
        }


    }
}
