package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ocv.SolvePNP;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

@TeleOp
@Config
public class PNPTest extends LinearOpMode {

    CameraSensor cameraSensor; SolvePNP solvePNP;

    MPPivot pivot;

    public static int exposure = 15;
    public static double angle = 10;
    private double lastExposure = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        int[] viewportids = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.HORIZONTAL);

        solvePNP = new SolvePNP();

        pivot = new MPPivot(hardwareMap);

        while(opModeInInit()) {
            pivot.setTargetAngle(angle);
            pivot.update();
        }

        cameraSensor = new CameraSensor(hardwareMap, "cam", telemetry, 0, false, true);
        cameraSensor.waitForSetExposure(3000, 10000, exposure);

        lastExposure = exposure;

        cameraSensor.setEnabled(true);

        while (opModeIsActive()) {
            cameraSensor.periodic();

            pivot.setTargetAngle(angle);
            pivot.update();

            if  (lastExposure!= exposure) {
                cameraSensor.waitForSetExposure(3000, 10000, exposure);
                lastExposure = exposure;
            }

            if (cameraSensor.detections()) {
                Vector2d sample = cameraSensor.getSamplePos();

                telemetry.addData("Sample X On Camera", sample.getX());
                telemetry.addData("Sample Y On Camera", sample.getY());
                telemetry.addLine();
                Pose offset = convertCameraCoordsToCenteredOffset(sample.getX(), sample.getY());
                telemetry.addData("Sample Offset X in inches", offset.getX());
                telemetry.addData("Sample Offset Y in inches", offset.getY());
                telemetry.addData("Sample Rotation Angle", cameraSensor.readSampleAngle());
                telemetry.addLine();
            }

            telemetry.update();
        }
    }

    public Pose targetOffsetSample(double sx, double sy) {
        Translation2d translation2d = new Translation2d(sx, sy);
        translation2d = solvePNP.camXYToInchOffset(translation2d.getX(), translation2d.getY());

        return new Pose(translation2d.getX(), translation2d.getY());
    }

    public Pose convertCameraCoordsToCenteredOffset(double cx, double cy) {
        double corneredCy = 480-cy;
        double centerRelativeX = cx-320;
        double centerRelativeY = corneredCy-240;
        return targetOffsetSample(centerRelativeX, centerRelativeY);
    }
}
