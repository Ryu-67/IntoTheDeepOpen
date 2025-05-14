package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.cmd.base.ArmCommand;
import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;


@TeleOp
@Config
public class DriveAndDeployAtSampleLocation extends LinearOpMode {

    PinpointDrive drive;
    CameraSensor cameraSensor;
    MPPivot pivot;
    SolvePNP solvePNP;

    Deposit deposit;

    public static double backOffset = 1, wrist = 0;

    boolean atSensedAngle = false, openSensor = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pivot = new MPPivot(hardwareMap);

        pivot.setTargetAngle(PNPTest.angle);

        solvePNP = new SolvePNP();

        deposit = new Deposit(hardwareMap);

        deposit.setDeposit(Deposit.hori, Deposit.aPitchBack, Deposit.armBack, Claw.open);

        cameraSensor = new CameraSensor(hardwareMap, "cam", telemetry, 0, false, false);
        cameraSensor.waitForSetExposure(3000, 10000, PNPTest.exposure);

        drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        double angle = 0;

        while(opModeInInit()) {
            if (pivot.check()) {
                    cameraSensor.periodic();
                    if (cameraSensor.detections()) {
                        angle = cameraSensor.readSampleAngle();
                        Vector2d sample = cameraSensor.getSamplePos();
                        telemetry.addData("Sample X On Camera", sample.getX());
                        telemetry.addData("Sample Y On Camera", sample.getY());
                        telemetry.addLine();
                        Pose2d offset = convertCameraCoordsToCenteredOffset(sample.getX(), sample.getY());
                        telemetry.addData("Sample Offset Y in inches", -offset.position.x);
                        telemetry.addData("Sample Offset X in inches", offset.position.y-backOffset);
                        telemetry.addData("Sample Rotation Angle", cameraSensor.readSampleAngle());
                        telemetry.addLine();
                        telemetry.update();
                    }
            }
            pivot.update();
        }

        deposit.setDeposit(wrist, Deposit.aPitchBack, Deposit.armBack, Claw.open);

        MecanumDrive.PARAMS.positionalError = 0.2;
        MecanumDrive.PARAMS.lateralVelGain = 25;
        MecanumDrive.PARAMS.lateralGain = 30;
        MecanumDrive.PARAMS.timeout = 0.1;

        Vector2d sample = cameraSensor.getSamplePos();
        Pose2d offset = convertCameraCoordsToCenteredOffset(sample.getX(), sample.getY());
        Pose2d correctedOffset = new Pose2d(offset.position.y-backOffset, -offset.position.x, Math.toRadians(0));

        Action a = drive.actionBuilder(new Pose2d(0,0,0))
                .setTangent(0)
                .lineToX(correctedOffset.position.x)
                                .setTangent(Math.toRadians(Math.signum(correctedOffset.position.y) * 90))
                                        .lineToY(correctedOffset.position.y).build();
        pivot.setTargetAngle(0);


        while (opModeIsActive()) {
           pivot.update();
           if (!a.run(new TelemetryPacket())) {
               deposit.setDeposit(wrist, Deposit.aPitchBack, Deposit.armBack, Claw.closed);
           }
        }
    }

    public Pose2d targetOffsetSample(double sx, double sy) {
        Translation2d translation2d = new Translation2d(sx, sy);
        translation2d = solvePNP.camXYToInchOffset(translation2d.getX(), translation2d.getY());

        return new Pose2d(translation2d.getX(), translation2d.getY(), 0);
    }

    public Pose2d convertCameraCoordsToCenteredOffset(double cx, double cy) {
        double corneredCy = 480-cy;
        double centerRelativeX = cx-320;
        double centerRelativeY = corneredCy-240;
        return targetOffsetSample(centerRelativeX, centerRelativeY);
    }
}
