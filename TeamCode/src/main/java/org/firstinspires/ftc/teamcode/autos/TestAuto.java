package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Macros;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@Autonomous
public class TestAuto extends LinearOpMode {

    SparkFunOTOSDrive drive; MainArm arm; Deposit deposit;

    TrajectoryActionBuilder initialDeposit, n1, n1Deposit, n2, n2Deposit, n3, n3Deposit, park;
    Action id, n1t, n1dt, n2t, n2dt, n3t, n3dt, pt;

    StateMachine fsm;
    StateMachine retraction;

    @Override
    public void runOpMode() {
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-40.5, -64.5, Math.toRadians(90)));
        arm = new MainArm(hardwareMap, false);
        deposit = new Deposit(hardwareMap);

        initialDeposit = drive.actionBuilder(new Pose2d(-40.5, -64.5, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-61, -60, Math.toRadians(45)), Math.PI);

        n1 = drive.actionBuilder(new Pose2d(-61, -60, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-49.5, -40.8, Math.toRadians(90)), Math.PI/2);

        n1Deposit = drive.actionBuilder(new Pose2d(-49.5, -40.8, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.PI);

        n2 = drive.actionBuilder(new Pose2d(-61, -60, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-59, -50, Math.toRadians(90)), Math.PI/2);

        n2Deposit = drive.actionBuilder(new Pose2d(-59, -50, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), Math.PI);

        n3 = drive.actionBuilder(new Pose2d(-60, -60, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-50, -48, Math.toRadians(128)), Math.PI/2);

        n3Deposit = drive.actionBuilder(new Pose2d(-50, -48, Math.toRadians(128)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), Math.PI*1.5);

        park = drive.actionBuilder(new Pose2d(-60, -60, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-28, -13, Math.toRadians(0)), 0);

        id = initialDeposit.build();
        n1t = n1.build();
        n1dt = n1Deposit.build();
        n2t = n2.build();
        n2dt = n2Deposit.build();
        n3t = n3.build();
        n3dt = n3Deposit.build();
        pt = park.build();

        ElapsedTime runtime = new ElapsedTime();

        fsm = new StateMachineBuilder()
                .state("id")
                .onEnter(()->{
                    arm.setLtg(2600);
                    deposit.setDeposit(Deposit.hori, 0.32, Deposit.armUp, Claw.closed);
                })
                .transition(()-> !id.run(new TelemetryPacket()) && arm.stateComplete(), runtime::reset)
                .state("drop")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open))
                .transitionTimed(0.5, "waitaround")
                .state("waitaround")
                .onEnter(()->{deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);})
                .transitionTimed(0.3,()->runtime.reset())
                .state("retract")
                .onEnter(()->arm.setLtg(0))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.6)
                .state("n1")
                        .onEnter(()->{
                            arm.setMainState(MainArm.State.intake);
                            deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);
                        })
                .transition(()-> !n1t.run(new TelemetryPacket()) && arm.stateComplete())
                .state("pickup")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed))
                .transitionTimed(0.3)
                .state("inter")
                    .onEnter(()->{
                        arm.setMainState(MainArm.State.basket);
                        deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed);
                    })
                .transition(()-> arm.stateComplete(), "n1dt")
                .state("n1dt")
                .transition(()->!n1dt.run(new TelemetryPacket()), runtime::reset)
                .state("lift")
                .onEnter(()->arm.setLtg(2600))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.4)
                .state("drop")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open))
                .build();

        arm.setMainState(MainArm.State.basket);


        while (!opModeIsActive() && opModeInInit()) {
            arm.update();
            if (arm.stateComplete()) {
                deposit.setDeposit(Deposit.hori, 0.79, 1, Claw.closed);
            }
        }
        deposit.setDeposit(Deposit.hori, Deposit.armUp, Deposit.aPitchUp, Claw.closed);

        fsm.start();

        while (opModeIsActive()) {
            fsm.update();
            arm.update();
            telemetry.addData("Current State", fsm.getStateString());
            telemetry.update();
        }



    }
}
