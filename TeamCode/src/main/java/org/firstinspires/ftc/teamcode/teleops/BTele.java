package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.cmd.basket.FlipAndExtendCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.DropAndResetCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.PickInCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.UpToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Basket Tele")
public class BTele extends OpMode {

    public Drive drive; public MPPivot pivot; public Deposit deposit; public Lift lift;

    public Robot robot;

    public CommandScheduler actionQueue = CommandScheduler.getInstance();

    boolean ip2 = false, isPr2 = false, irp = false, wasLastPick = false, ih = false, hang = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true, false, true);
        drive = robot.drive;
        lift = robot.lift;
        deposit = robot.deposit;
        pivot = robot.pivot;
        pivot.setTargetAngle(0);
        lift.setTarget(0);

        actionQueue.reset();
        actionQueue = CommandScheduler.getInstance();
    }

    @Override
    public void loop() {
        drive.setFlags(gamepad1.left_bumper, gamepad1.right_bumper);
        drive.setXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        deposit.process(gamepad2.dpad_up, gamepad2.dpad_down);
        lift.setManualInput(gamepad2.left_stick_x);

        if (!ip2 && gamepad2.x) {
            pivot.setTargetAngle(92);
        } else if (!ip2 && gamepad2.y) {
            pivot.setTargetAngle(MainArm.hangle);
        } else if (!ip2 && gamepad2.b) {
            pivot.setTargetAngle(0);
        } else if (!ip2 && gamepad2.a) {
            pivot.setTargetAngle(95);
        } else if (!ip2 && gamepad2.left_stick_button) {
            pivot.setTargetAngle(30);
        }
        ip2 = gamepad2.b || gamepad2.x || gamepad2.y || gamepad2.a || gamepad2.left_stick_button;

        pivot.setTargetAngle(Math.toDegrees(pivot.targetAngle) - gamepad2.right_stick_y);

        actionQueue.run();

        if (gamepad1.dpad_up && !wasLastPick) {
            Deposit.armBack += 0.02;
        } wasLastPick = gamepad1.dpad_up;

        drive.update();
        lift.update();
        pivot.update();
        deposit.update(gamepad2.left_bumper, gamepad2.right_bumper);

        if (gamepad2.dpad_left && !isPr2 && pivot.angle <= Math.toRadians(45)) {
            actionQueue.schedule(new PickInCommand(deposit, lift));
        } else if (gamepad2.dpad_left && !isPr2 && pivot.targetAngle >= Math.toRadians(45)) {
            actionQueue.schedule(new DropAndResetCommand(deposit, lift, pivot));
            deposit.incr = 1;
        } else if (gamepad2.a && !isPr2 && pivot.angle <= Math.toRadians(45)) {
            actionQueue.schedule(new UpToCommand(lift, pivot));
        }
        isPr2 = gamepad2.dpad_left || gamepad2.a;

        if (gamepad2.dpad_right && !irp) {
            if (lift.ticks > 250) {
                lift.setTarget(0);
            } else {
                lift.setTarget(873);
            }
        } irp = gamepad2.dpad_right;

        lift.setLimit(pivot.angle > Math.toRadians(20));

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("slide reset switch", lift.readSensor());
        telemetry.addData("Lift Ticks", lift.getTicks());
        telemetry.addData("Lift Target", lift.target);
        telemetry.update();
    }
}
