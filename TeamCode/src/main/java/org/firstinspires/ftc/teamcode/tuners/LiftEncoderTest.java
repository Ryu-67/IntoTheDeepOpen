package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp
public class LiftEncoderTest extends OpMode {

    private DcMotor lLift;

    private RevTouchSensor lReset;

    @Override
    public void init() {
        lLift = hardwareMap.dcMotor.get("lLift");
        lLift.setDirection(DcMotorSimple.Direction.REVERSE);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lReset = hardwareMap.get(RevTouchSensor.class, "lReset");
    }

    @Override
    public void loop() {
        telemetry.addData("encoder pose", Math.abs(lLift.getCurrentPosition()));
        telemetry.addData("reset", lReset.isPressed());
        telemetry.update();
    }
}
