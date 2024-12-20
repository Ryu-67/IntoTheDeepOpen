package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@Config
@Disabled
@Autonomous(name = "Push Specimen Auto", preselectTeleOp = "Specimen Tele")
public class SpecAuto extends LinearOpMode {

    SparkFunOTOSDrive drive; MainArm arm; Deposit deposit;

    TrajectoryActionBuilder initialDeposit, triplash, backin, n1, n1deposit, n2, n2Deposit, n3, n3Deposit, park;
    Action id, back, trip;

    StateMachine fsm;

    public static double upticks = 1000, waitTime = 0.8, y = -37.8;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(7.4, -63.5, Math.toRadians(-90)));
        arm = new MainArm(hardwareMap, false);
        deposit = new Deposit(hardwareMap);

        initialDeposit = drive.actionBuilder(new Pose2d(7.4, -63.5, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .lineToY(y);

        triplash = drive.actionBuilder(new Pose2d(7.4, y, Math.toRadians(-90)))
                .setTangent(0)
                .lineToX(24)
                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(-90)), Math.toRadians(20))
                .setTangent(Math.toRadians(90))
                .lineToY(-51)
                .waitSeconds(0.001)
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(-10)
                .waitSeconds(0.01)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(58, -51), Math.toRadians(70));

        id = initialDeposit.build();
        trip = triplash.build();

        fsm = new StateMachineBuilder()
                .state("id")
                .onEnter(()->{
                    arm.setLtg(upticks);
                })
                .transition(()->!id.run(new TelemetryPacket()) && arm.stateComplete(), "rest")
                .state("rest")
                .onEnter(()->arm.setLtg(2200))
                .transitionTimed(0.75)
                .state("release")
                .onEnter(()->{
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.open);
                    arm.setLtg(0);
                })
                .transition(()->arm.stateComplete() && !trip.run(new TelemetryPacket()))
                .state("hey")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open);
                })
                .build();

        arm.setMainState(MainArm.State.basket);

        while (!opModeIsActive() && opModeInInit()) {
            arm.update();
            drive.otos.setPosition(new SparkFunOTOS.Pose2D(7.4, -63.5, Math.toRadians(-90)));
        }

        fsm.start();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();
        double i = 0, i1 = 0;

        deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.closed);

        while (opModeIsActive()) {
            i1 = loopTimer.milliseconds();
            arm.update();
            fsm.update();
            telemetry.addData("Current State", fsm.getStateString());
            telemetry.addData("Looptime", (i1-i));
            telemetry.update();
            i = i1;
        }
    }
}
