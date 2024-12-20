package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@Config
@Autonomous(name = "1+2 Specimen Auto", preselectTeleOp = "Specimen Tele")
public class ThreeNPush extends LinearOpMode {

    SparkFunOTOSDrive drive; MainArm arm; Deposit deposit;

    TrajectoryActionBuilder initialDeposit, triplash, n1, n1deposit, n2, n2deposit, f1, f2, park, f0;
    Action id, trip, pick1, d1, pt, pick2, d2, f1t, f2t, f0t;

    StateMachine fsm;

    public static double upticks = 970, waitTime = 0.8, y = -35;

    ElapsedTime t = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(7.4, -63.5, Math.toRadians(-90)));
        arm = new MainArm(hardwareMap, false);
        deposit = new Deposit(hardwareMap);

        initialDeposit = drive.actionBuilder(new Pose2d(7.4, -63.5, Math.toRadians(-90)))
                .waitSeconds(0.6)
                .setTangent(Math.toRadians(90))
                .lineToY(y);

        f0 = drive.actionBuilder(new Pose2d(7.4, y, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-34)
                .waitSeconds(0.2);

        triplash = drive.actionBuilder(new Pose2d(7.4, -34, Math.toRadians(-90)))
                .waitSeconds(0.01)
                .setTangent(90)
                .lineToY(-35)
                .setTangent(Math.toRadians(0))
                .lineToX(25)
                .splineToSplineHeading(new Pose2d(43, -12, Math.toRadians(-90)), Math.toRadians(20))
                .setTangent(Math.toRadians(90))
                .lineToY(-54)
                .waitSeconds(0.001)
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(-10)
                .waitSeconds(0.01)
                .setTangent(Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(56, -45), Math.toRadians(-90));

        n1 = drive.actionBuilder(new Pose2d(56, -45, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-55.5);

        n1deposit = drive.actionBuilder(new Pose2d(56, -55.5, Math.toRadians(-90)))
                .setTangent(0)
                .lineToXSplineHeading(25, Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-2, -34.2), Math.toRadians(90));

        f1 = drive.actionBuilder(new Pose2d(-2, -34.2, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(0.25);


        n2 = drive.actionBuilder(new Pose2d(-2, -33, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38, -45), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(-54.5);

        n2deposit = drive.actionBuilder(new Pose2d(38, -54.5, Math.toRadians(-90)))
                .setTangent(0)
                .lineToXSplineHeading(25, Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-8, -33.8), Math.toRadians(90));

        f2 = drive.actionBuilder(new Pose2d(-8, -33.8, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .lineToY(-33)
                .waitSeconds(0.25);

        park = drive.actionBuilder(new Pose2d(-8, -33, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(0));


        id = initialDeposit.build();
        trip = triplash.build();
        pick1 = n1.build();
        d1 = n1deposit.build();
        pt = park.build();
        pick2 = n2.build();
        d2 = n2deposit.build();
        f1t = f1.build();
        f2t = f2.build();
        f0t = f0.build();

        fsm = new StateMachineBuilder()
                .state("id")
                .onEnter(()->{
                    arm.setLtg(1010);
                })
                .transition(()->!id.run(new TelemetryPacket()) && arm.stateComplete(), "rest")
                .state("rest")
                .onEnter(()->arm.setLtg(0))
                .transition(()->!f0t.run(new TelemetryPacket()), "release")
                .state("release")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.wrapped, 0.55, 0.8, Claw.open);
                    arm.setLtg(0);
                })
                .transition(()->arm.stateComplete() && !trip.run(new TelemetryPacket()))
                .state("inter")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.wrapped, 0.55, 0.8, Claw.open);
                })
                .transition(()->arm.stateComplete())
                .state("hey")
                .transition(()->!pick1.run(new TelemetryPacket()))
                .state("close")
                .onEnter(()->deposit.setDeposit(Deposit.wrapped, 0.55, 0.8, Claw.closed))
                .transitionTimed(0.35, "Armup", ()->t.reset())
                .state("Armup")
                .onEnter(()->{arm.setMainState(MainArm.State.basket); deposit.setDeposit(Deposit.wrapped, Deposit.aPitchDown, Deposit.armDown, Claw.closed);})
                .transition(()->arm.stateComplete() && t.seconds() > 0.6)
                .state("depo2")
                .onEnter(()-> {
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.closed);
                    arm.setLtg(980);
                })
                .transition(()->!d1.run(new TelemetryPacket()) && arm.stateComplete())
                .state("res2t")
                .onEnter(()->arm.setLtg(0))
                .transition(()->!f1t.run(new TelemetryPacket()), "release2")
                .state("release2")
                .onEnter(()->{
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.open);
                })
                .transition(()->arm.stateComplete())
                .state("reset")
                .onEnter(()-> {
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.wrapped, 0.55, 0.8, Claw.open);
                })
                .transition(()->arm.stateComplete() && !pick2.run(new TelemetryPacket()))
                .transition(()->!pick2.run(new TelemetryPacket()))
                .state("close2")
                .onEnter(()->deposit.setDeposit(Deposit.wrapped, 0.55, 0.8, Claw.closed))
                .transitionTimed(0.45, "Armup2", ()->t.reset())
                .state("Armup2")
                .onEnter(()->{arm.setMainState(MainArm.State.basket); deposit.setDeposit(Deposit.wrapped, Deposit.aPitchDown, Deposit.armDown, Claw.closed);})
                .transition(()->arm.stateComplete() && t.seconds() > 0.6)
                .state("depo3")
                .onEnter(()-> {
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.closed);
                    arm.setLtg(980);
                })
                .transition(()->!d2.run(new TelemetryPacket()) && arm.stateComplete())
                .state("rest3")
                .onEnter(()->arm.setLtg(0))
                .transition(()->!f2t.run(new TelemetryPacket()), "release3")
                .state("release3")
                .onEnter(()->{
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchDown, Deposit.armDown, Claw.open);
                })
                .transition(()->arm.stateComplete())
                .state("downnpark")
                .onEnter(()-> {
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.hori, 0.55, 0.8, Claw.open);
                })
                .transition(()->arm.stateComplete() && !pt.run(new TelemetryPacket()))
                .state("pk")
                .transition(()->!pt.run(new TelemetryPacket()))
                .state("fin")
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

        deposit.setDeposit(Deposit.hori, 0.88, Deposit.armDown, Claw.closed);

        while (opModeIsActive()) {
            i1 = loopTimer.milliseconds();
            arm.update();
            fsm.update();
            telemetry.addData("Current State", fsm.getStateString());
            telemetry.addData("Looptime", 1000/(i1-i));
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.update();
            i = i1;
        }
    }
}
