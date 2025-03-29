package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;

@TeleOp
@Config
public class ColorDetectionTest extends LinearOpMode {

    CameraSensor cameraSensor;
    public static int exposure = 30;

    @Override
    public void runOpMode() throws InterruptedException {
//        int[] viewportids = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.HORIZONTAL);
        cameraSensor = new CameraSensor(hardwareMap, "cam", telemetry, 0, false, true);
        cameraSensor.waitForSetExposure(3000, 10000, exposure);

        waitForStart();

        cameraSensor.setEnabled(true);


        while (opModeIsActive()) {
            telemetry.update();
            cameraSensor.periodic();
        }
    }
}
