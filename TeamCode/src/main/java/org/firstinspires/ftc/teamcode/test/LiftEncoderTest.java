package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftEncoderTest extends OpMode {

    private DcMotor lLift;

    private RevTouchSensor lReset;

    @Override
    public void init() {
        lLift = hardwareMap.dcMotor.get("lLift");
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lReset = hardwareMap.get(RevTouchSensor.class, "lReset");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("encoder pose", Math.abs(lLift.getCurrentPosition()));
        telemetry.addData("reset", lReset.isPressed());
        telemetry.update();
    }
}
