package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.autos.cmd.basket.VisionGrabCommand;
import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@Autonomous
public class GamepadMenuFTCLibTest extends LinearOpMode {
    CameraSensor cameraSensor;
    MPPivot pivot; SolvePNP solvePNP; Deposit deposit; PinpointDrive drive;
    CommandScheduler s = CommandScheduler.getInstance();
    public boolean vert = false;
    public boolean ip = false;

    private double wristP = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        pivot = new MPPivot(hardwareMap); pivot.setTargetAngle(30); solvePNP = new SolvePNP();
        deposit = new Deposit(hardwareMap);
        deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);

        cameraSensor = new CameraSensor(hardwareMap, "cam", telemetry, 0, false, false);


        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        s.reset(); s = CommandScheduler.getInstance();

        cameraSensor.startStreaming();
        cameraSensor.waitForSetExposure(3000, 10000, PNPTest.exposure);
        cameraSensor.setEnabled(true);

        while (opModeInInit()) {
            pivot.update();

            if (gamepad2.left_bumper && !ip) {
                vert = !vert;
            } ip = gamepad2.left_bumper;

            telemetry.addData("Is Sample Vertical?", vert);
            telemetry.update();
        }

        if (vert) {
            wristP = 0.5;
        }

        s.schedule(new VisionGrabCommand(cameraSensor, deposit, pivot, drive, wristP, telemetry));

        while (opModeIsActive()) {
            pivot.update();
            s.run();
        }
    }
}
