package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hang;

@TeleOp
@Config
public class WinchHangTest extends OpMode {

    Hang hang;
    public static double inputPower = 0;

    @Override
    public void init() {
        hang = new Hang(hardwareMap);
    }

    @Override
    public void loop() {
        hang.setPower(inputPower);
    }
}
