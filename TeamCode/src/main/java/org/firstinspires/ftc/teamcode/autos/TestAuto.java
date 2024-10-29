package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.rr.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@Autonomous
public class TestAuto extends LinearOpMode {

    SparkFunOTOSDrive drive; MainArm arm; Arm deposit;

    TrajectoryActionBuilder initialDeposit, n1, n1Deposit, n2, n2Deposit, n3, n3Deposit, park;
    Action id, n1t, n1dt, n2t, n2dt, n3t, n3dt, pt;

    StateMachine fsm;

    @Override
    public void runOpMode() {
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(-40.5, -64.5, Math.toRadians(90)));
        arm = new MainArm(hardwareMap, false);
        deposit = new Arm(hardwareMap);

        initialDeposit = drive.actionBuilder(new Pose2d(-40.5, -64.5, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), Math.PI);

        n1 = drive.actionBuilder(new Pose2d(-60, -60, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-59, -50, Math.toRadians(63)), Math.PI/2);

        n1Deposit = drive.actionBuilder(new Pose2d(-59, -50, Math.toRadians(63)))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), Math.PI);

        n2 = drive.actionBuilder(new Pose2d(-60, -60, Math.toRadians(45)))
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

        fsm = new StateMachineBuilder()
                .state("n1")
                        .onEnter(()->{
                            arm.setMainState(MainArm.State.intake);
                            arm.setLtg(Globals.n1Extension);
                            deposit.setDeposit(Arm.hori, Arm.aPitchBack, Arm.aBack, Claw.open);
                        })
                                .transition(()-> arm.stateComplete() && !id.run(new TelemetryPacket()))
                .build();

        waitForStart();

        fsm.start();

        while (opModeIsActive() && fsm.isRunning()) {
            fsm.update();
            arm.update();
        }



    }
}
