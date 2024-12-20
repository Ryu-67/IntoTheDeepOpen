package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

@TeleOp(name = "One Driver Basket Tele")
@Disabled
public class SoloBasket extends OpMode {

    public Drive drive;
    public MainArm arm;
    public Deposit deposit;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        arm = new MainArm(hardwareMap, true);
        deposit = new Deposit(hardwareMap);

        arm.setMainState(MainArm.State.intake);
    }
    boolean ip2 = false;


    @Override
    public void loop() {
        drive.setXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        deposit.process(gamepad1.b, gamepad1.x);


        arm.setLmi((gamepad1.right_trigger - gamepad1.left_trigger));

        if (!ip2 && gamepad1.dpad_left) {
            arm.setMainState(MainArm.State.basket);
        } else if (!ip2 && gamepad1.dpad_up) {
            arm.setMainState(MainArm.State.hang);
        } else if (!ip2 && gamepad1.dpad_right) {
            arm.setMainState(MainArm.State.intake);
        }
        ip2 = gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right;

        drive.update();
        deposit.update(gamepad1.left_bumper, gamepad1.right_bumper);
        arm.update();
    }
}