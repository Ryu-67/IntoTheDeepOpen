package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;

@TeleOp
@Config
public class ArmT extends OpMode {

    Deposit arm;

    public static double armPos, wristPos, pitch, claw;

    @Override
    public void init() {
        arm = new Deposit(hardwareMap);
    }

    @Override
    public void loop() {
        arm.setDeposit(wristPos, pitch, armPos, claw);
    }
}
