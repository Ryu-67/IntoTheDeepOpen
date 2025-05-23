package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autos.cmd.StartStream;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.ConfigArmCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.LiftLimitCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PivotCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.RRFollowCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.ReturnFromCycleCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.VisionGrabCommand;
import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.test.PNPTest;

@Autonomous(name= "Roadrunner Sub Auton")
public class RRCycleSamp extends LinearOpMode {

    Lift lift; MPPivot pivot; Deposit deposit; PinpointDrive pinpointDrive; CameraSensor cameraSensor; SolvePNP solvePNP;

    Action preload, grab1, drop1, grab2, drop2, grab3, drop3, toSub;

    CommandScheduler s;

    public static double dropTiming = 0.2;
    public static long flipTiming = 520;

    public boolean vert = false, ip = false, ip2 = false;

    private double wristP = 0, cycleIncrement = 0;

    Pose2d startpose = new Pose2d(-47+7.5, -72+7.4, Math.toRadians(0)),
            forwardPose = new Pose2d(-47+7.4, -58, Math.toRadians(90)),
            preloadPose = new Pose2d(-60, -57, Math.toRadians(45)),
            grab1Pose = new Pose2d(-51.2, -41.2, Math.toRadians(90)),
            drop1Pose = new Pose2d(-60, -58, Math.toRadians(45)),
            grab2Pose = new Pose2d(-62.5, -41.2, Math.toRadians(90)),
            drop2Pose = new Pose2d(-57, -62, Math.toRadians(45)),
            grab3pose = new Pose2d(-63.6, -40.4, Math.toRadians(122.5)),
            drop3Pose = new Pose2d(-57, -62, Math.toRadians(45)),
            subPose = new Pose2d(-20, -9, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, false);
        pinpointDrive = new PinpointDrive(hardwareMap, startpose);
        pivot = new MPPivot(hardwareMap);
        deposit = new Deposit(hardwareMap);
        solvePNP = new SolvePNP();

        cameraSensor = new CameraSensor(hardwareMap, "cam", telemetry, 0, false, false);
        cameraSensor.startStreaming();
        cameraSensor.waitForSetExposure(3000, 10000, PNPTest.exposure);

        pivot.setTargetAngle(89.5);
        lift.setTarget(0);

        preload = pinpointDrive.actionBuilder(startpose).setTangent(Math.toRadians(90))
                .setTangent(Math.toRadians(135))
                .lineToYConstantHeading(-58)
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(preloadPose, Math.toRadians(140)).build();

        grab1 = pinpointDrive.actionBuilder(preloadPose).setTangent(Math.toRadians(45))
                .splineToSplineHeading(grab1Pose, Math.toRadians(90)).build();

        drop1 = pinpointDrive.actionBuilder(grab1Pose).setTangent(Math.toRadians(-90))
                .splineToSplineHeading(drop1Pose, Math.toRadians(-135)).build();

        grab2 = pinpointDrive.actionBuilder(drop1Pose).setTangent(Math.toRadians(110))
                .splineToSplineHeading(grab2Pose, Math.toRadians(90)).build();

        drop2 = pinpointDrive.actionBuilder(grab2Pose).setTangent(Math.toRadians(-30))
                .splineToSplineHeading(drop2Pose, Math.toRadians(-135)).build();

        grab3 = pinpointDrive.actionBuilder(drop2Pose).setTangent(Math.toRadians(110))
                .splineToSplineHeading(grab3pose, Math.toRadians(120)).build();

        drop3 = pinpointDrive.actionBuilder(grab3pose).setTangent(Math.toRadians(-60))
                .splineToSplineHeading(drop3Pose, Math.toRadians(-135)).build();

        s = CommandScheduler.getInstance();
        s.reset();
        s = CommandScheduler.getInstance();


        while (opModeInInit()) {
            pivot.update();
            deposit.setDeposit(0,0,0,0);
            lift.update();

            if (gamepad2.left_bumper && !ip) {
                vert = !vert;
            } ip = gamepad2.left_bumper;

            if (gamepad2.dpad_up && !ip2) {
                cycleIncrement += 1;
            } else if (gamepad2.dpad_down && !ip2) {
                cycleIncrement -= 1;
            } ip2 = gamepad2.dpad_down || gamepad2.dpad_up;

            telemetry.addData("Is Sample Vertical?", vert);
            telemetry.addData("Vertical Cycle Offset", cycleIncrement);
            telemetry.update();
        }

        if (vert) {
            wristP = 0.5;
        }

        s.schedule(
                new LiftLimitCommand(lift, true),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new RRFollowCommand(preload, 0.2),
                                new ParallelCommandGroup(
                                        new LiftCommand(lift, 870),
                                        new SequentialCommandGroup(
                                                new WaitCommand(flipTiming),
                                                new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                ),
                                new RRFollowCommand(grab1, 0.3)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new RRFollowCommand(drop1, 0.2),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 89.5),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 870),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(flipTiming),
                                                        new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                                                )
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                ),
                                new RRFollowCommand(grab2, 0.3)
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.extralowground, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal),
                        new ParallelCommandGroup(
                                new RRFollowCommand(drop2, 0.2),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 89.5),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 870),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(flipTiming),
                                                        new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                                                )
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.diagonal2, dropTiming),
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 0)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new RRFollowCommand(grab3, 0.1)
                                )
                        ),
                        new ArmCommand(deposit, ArmCommand.DepositState.floorIntake, ArmCommand.ClawState.closed, ArmCommand.WristState.diagonal2),
                        new ParallelCommandGroup(
                                new RRFollowCommand(drop3, 0.2),
                                new SequentialCommandGroup(
                                        new PivotCommand(pivot, 89.5),
                                        new ParallelCommandGroup(
                                                new LiftCommand(lift, 870),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(flipTiming),
                                                        new ArmCommand(deposit, ArmCommand.DepositState.basketDepo, ArmCommand.ClawState.closed, ArmCommand.WristState.horizontal)
                                                )
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                                        new LiftCommand(lift, 0),
                                        new PivotCommand(pivot, 30)
                                ),
                                new RRFollowCommand(pinpointDrive.actionBuilder(drop3Pose).setTangent(Math.toRadians(70))
                                        .splineToSplineHeading(new Pose2d(-19, -9+cycleIncrement, Math.toRadians(0)), Math.toRadians(0)).build(), 0)
                        ),
                        new StartStream(cameraSensor, true),
                        new VisionGrabCommand(cameraSensor, deposit, pivot, pinpointDrive, wristP, telemetry),
                        new StartStream(cameraSensor, false),
                        new WaitCommand(250),
                        new SequentialCommandGroup(
                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.closed, ConfigArmCommand.WristState.horizontal, 0.25),
                                new ParallelCommandGroup(
                                        new ReturnFromCycleCommand(pinpointDrive, 0.3),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1200),
                                                new PivotCommand(pivot, 89.5),
                                                new LiftCommand(lift, 870)
                                        )
                                )
                        ),
                        new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.basketDepo, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                        new SequentialCommandGroup(
                                new ConfigArmCommand(deposit, ConfigArmCommand.DepositState.floorIntake, ConfigArmCommand.ClawState.open, ConfigArmCommand.WristState.horizontal, dropTiming),
                                new LiftCommand(lift, 0),
                                new PivotCommand(pivot, 0)
                        )
                )
        );


        while (opModeIsActive()) {
            s.run();
            lift.update();
            pivot.update();
            telemetry.addData("Liftpos", lift.ticks);
            telemetry.update();
        }
    }
}
