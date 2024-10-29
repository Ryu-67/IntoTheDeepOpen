package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Disabled
@TeleOp
public class ArmT extends OpMode {

    Arm arm;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
    }

    @Override
    public void loop() {
        arm.process(gamepad1.dpad_down, gamepad1.dpad_up);
        arm .update(gamepad1.left_bumper, gamepad1.right_bumper);
    }
}
