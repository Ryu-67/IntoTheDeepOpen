package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@Autonomous(name="Basket Auto", preselectTeleOp = "Basket Tele")
public class BasketAuto extends LinearOpMode {

    SparkFunOTOSDrive drive; MainArm arm; Deposit deposit;

    TrajectoryActionBuilder initialDeposit, n1, n1Deposit, n2, n2Deposit, n3, n3Deposit, park;
    Action id, n1t, n1dt, n2t, n2dt, n3t, n3dt, pt;

    StateMachine fsm;

    @Override
    public void runOpMode() {
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-40.5, -64.5, Math.toRadians(90)));
        arm = new MainArm(hardwareMap, false);
        deposit = new Deposit(hardwareMap);

        initialDeposit = drive.actionBuilder(new Pose2d(-40.5, -64.5, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-59.5, -59, Math.toRadians(45)), Math.toRadians(-120));

        n1 = drive.actionBuilder(new Pose2d(-59.5, -59, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-49.5, -41, Math.toRadians(92)), Math.PI/2);

        n1Deposit = drive.actionBuilder(new Pose2d(-49.5, -41, Math.toRadians(92)))
                .splineToLinearHeading(new Pose2d(-58.25, -58.25, Math.toRadians(45)), 3*Math.PI);

        n2 = drive.actionBuilder(new Pose2d(-58.25, -58.25, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-60.15, -41, Math.toRadians(90)), Math.PI/2);

        n2Deposit = drive.actionBuilder(new Pose2d(-60.15, -41, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-59.5, -59.5, Math.toRadians(45)), Math.PI);

        n3 = drive.actionBuilder(new Pose2d(-59.5, -59.5, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-58.65, -36.25, Math.toRadians(140)), Math.PI);

        n3Deposit = drive.actionBuilder(new Pose2d(-58.65, -35.85, Math.toRadians(140)))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-57.8, -57.8, Math.toRadians(45)), Math.toRadians(180));

        park = drive.actionBuilder(new Pose2d(-57.8, -57.8, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-33, -13, Math.toRadians(0)), 0);

        id = initialDeposit.build();
        n1t = n1.build();
        n1dt = n1Deposit.build();
        n2t = n2.build();
        n2dt = n2Deposit.build();
        n3t = n3.build();
        n3dt = n3Deposit.build();
        pt = park.build();

        ElapsedTime runtime = new ElapsedTime();

        ElapsedTime matchTime = new ElapsedTime();

        fsm = new StateMachineBuilder()
                .state("id")
                .onEnter(()->{
                    arm.setLtg(2600);
                })
                .transition(()-> !id.run(new TelemetryPacket()) && arm.stateComplete(), "wait",()-> {runtime.reset(); deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed);})
                .transition(()-> matchTime.seconds() > 25, "fine")
                .state("wait")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .transitionTimed(0.3, "drop")
                .state("drop")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open))
                .transitionTimed(0.2, "waitaround")
                .state("waitaround")
                .onEnter(()->{deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);})
                .transitionTimed(0.3, "retract", ()->runtime.reset())
                .state("retract")
                .onEnter(()->arm.setLtg(0))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.6, "n1")
                .state("n1")
                        .onEnter(()->{
                            arm.setMainState(MainArm.State.intake);
                            deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);
                        })
                .transition(()-> !n1t.run(new TelemetryPacket()) && arm.stateComplete(), "pickup")
                .state("pickup")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed))
                .transitionTimed(0.3, "inter")
                .state("inter")
                    .onEnter(()->{
                        arm.setMainState(MainArm.State.basket);
                    })
                .transition(()-> arm.stateComplete(), "n1dt")
                .state("n1dt")
                .transition(()->!n1dt.run(new TelemetryPacket()),"lift", ()->{runtime.reset(); deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed);})
                .state("lift")
                .onEnter(()->arm.setLtg(2600))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.4, "iforgot", ()->{runtime.reset(); deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed);})
                .state("iforgot")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .transitionTimed(0.56, "drop2")
                .state("drop2")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open))
                .transitionTimed(0.2, "holllllllddddd")
                .state("holllllllddddd")
                .onEnter(()->{deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);})
                .transitionTimed(0.2, "retract2",()->runtime.reset())
                .state("retract2")
                .onEnter(()->arm.setLtg(0))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.6, "n2")
                .state("n2")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);
                })
                .transition(()-> !n2t.run(new TelemetryPacket()) && arm.stateComplete(), "pickup2")
                .state("pickup2")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed))
                .transitionTimed(0.3, "inter2")
                .state("inter2")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.basket);
                })
                .transition(()-> arm.stateComplete(), "n2dt")
                .state("n2dt")
                .transition(()->!n2dt.run(new TelemetryPacket()), "lift2",runtime::reset)
                .state("lift2")
                .onEnter(()->arm.setLtg(2600))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.4, "wait2",()->{runtime.reset(); deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.closed);})
                .state("wait2")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .transitionTimed(0.56, "drop3")
                .state("drop3")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open))
                .transitionTimed(0.2, "hold for the love of god")
                .state("hold for the love of god")
                .onEnter(()->{deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);})
                .transitionTimed(0.2, "retract3",()->runtime.reset())
                .state("retract3")
                .onEnter(()->arm.setLtg(0))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.6, "n3")
                .state("n3")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                    deposit.setDeposit(0.6, Deposit.aPitchBack, Deposit.armBack, Claw.open);
                })
                .transition(()-> !n3t.run(new TelemetryPacket()) && arm.stateComplete(), "pickup3")
                .state("pickup3")
                .onEnter(()->deposit.setDeposit(0.6, Deposit.aPitchBack, Deposit.armBack, Claw.closed))
                .transitionTimed(0.3, "inter3")
                .state("inter3")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.basket);
                })
                .transition(()-> arm.stateComplete(), "n3dt")
                .state("n3dt")
                .transition(()->!n3dt.run(new TelemetryPacket()), "lift3",runtime::reset)
                .state("lift3")
                .onEnter(()->arm.setLtg(2600))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.4, "wait3", ()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .state("wait3")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.closed))
                .transitionTimed(0.8, "drop4")
                .state("drop4")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open))
                .transitionTimed(0.5, "pullback")
                .onEnter(()->deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open))
                .transitionTimed(0.4, "retract4",()->runtime.reset())
                .state("retract4")
                .onEnter(()->arm.setLtg(0))
                .transition(()->arm.stateComplete() && runtime.seconds() > 0.6, "park")
                .state("park")
                .onEnter(()->{
                    arm.setMainState(MainArm.State.intake);
                })
                .transition(()->!pt.run(new TelemetryPacket()), "fine")
                .state("fine")
                .onEnter(()->{
                    deposit.setDeposit(Deposit.hori, Deposit.aPitchUp, Deposit.armUp, Claw.open);
                    arm.setMainState(MainArm.State.intake);
                    arm.setLtg(0);
                })
                .build();

        arm.setMainState(MainArm.State.basket);


        while (!opModeIsActive() && opModeInInit()) {
            arm.update();
            drive.otos.setPosition(new SparkFunOTOS.Pose2D(-40.5, -64.5, Math.toRadians(90)));
        }
        deposit.setDeposit(Deposit.hori, Deposit.armUp, Deposit.aPitchUp, Claw.closed);

        fsm.start();
        matchTime.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();
        double i = 0, i1 = 0;

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
