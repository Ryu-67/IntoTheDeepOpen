package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.cmd.basket.FlipAndExtendCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.DropAndResetCommand;
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

    boolean ip2 = false, isPr2 = false, irp = false;

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

        if (!ip2 && gamepad2.b) {
            pivot.setTargetAngle(0);
            lift.setLimit(false);
        } else if (!ip2 && gamepad2.y) {
            pivot.setTargetAngle(MainArm.hangle);
            lift.setLimit(true);
        } else if (!ip2 && gamepad2.x) {
            pivot.setTargetAngle(90);
            lift.setLimit(true);
        }
        ip2 = gamepad2.b || gamepad2.y || gamepad2.x;

        actionQueue.run();

        drive.update();
        lift.update();
        pivot.update();
        deposit.update(gamepad2.left_bumper, gamepad2.right_bumper);

        if (gamepad2.dpad_left && !isPr2 && Math.toDegrees(pivot.angle) < MainArm.hangle) {
            actionQueue.schedule(new FlipAndExtendCommand(deposit, lift, pivot));
            deposit.incr = 2;
        } else if (gamepad2.dpad_left && !isPr2 && Math.toDegrees(pivot.angle) > MainArm.hangle) {
            actionQueue.schedule(new DropAndResetCommand(deposit, lift, pivot));
            deposit.incr = 1;
        } isPr2 = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !irp) {
            if (lift.ticks > 500) {
                lift.setTarget(0);
            } else {
                lift.setTarget(Lift.limit);
            }
        } irp = gamepad2.dpad_right;

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("slide reset switch", lift.readSensor());
        telemetry.addData("Lift Ticks", lift.getTicks());
        telemetry.addData("Lift Target", lift.target);
        telemetry.update();
    }
}