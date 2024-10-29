package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Disabled
@TeleOp
@Config
public class ClawTuner extends OpMode {

    private Claw claw;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
    }

    @Override
    public void loop() {
        claw.update(gamepad1.a);
    }
}
