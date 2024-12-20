package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.cmd.spec.PickAndFlipCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.SpecSlamCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Mat;

@TeleOp(name = "Specimen Tele")
public class SpecTele extends OpMode {

    public Drive drive; public MPPivot pivot; public Deposit deposit; public Lift lift;
    public Robot robot;

    CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true, true, true);
        drive = robot.drive;
        pivot = robot.pivot;
        deposit = robot.deposit;
        lift = robot.lift;

        pivot.setTargetAngle(0);

        actionQueue.reset();
        actionQueue = CommandScheduler.getInstance();

        actionQueue.reset();
    }

    boolean hang = false; boolean irp = false, ip2 = false, ip3 = false, g = false, isPr = false;
    double m = 1;

    @Override
    public void loop() {

        if (gamepad1.a && !ip3) {
            if (m == 1) {
                m = -1;
            } else {
                m=1;
            }
            g = !g;
        } ip3 = gamepad1.a;

        drive.setFlags(gamepad1.left_bumper, gamepad1.right_bumper);
        drive.setXYZ(gamepad1.left_stick_x * m, gamepad1.left_stick_y * m, gamepad1.right_stick_x * m);
        lift.setManualInput(gamepad2.left_stick_x);

        if (!ip2 && gamepad2.x) {
            pivot.setTargetAngle(MainArm.vertAngle);
            lift.setLimit(true);
        } else if (!ip2 && gamepad2.y) {
            lift.setLimit(true);
            pivot.setTargetAngle(MainArm.hangle);
        } else if (!ip2 && gamepad2.b) {
            lift.setLimit(false);
            pivot.setTargetAngle(0);
        }
        ip2 = gamepad2.b || gamepad2.a || gamepad2.x;
        deposit.process(gamepad2.dpad_up, gamepad2.dpad_down);

        robot.update(gamepad2.left_bumper, gamepad2.right_bumper);
        actionQueue.run();

        if (gamepad2.dpad_left && !isPr && pivot.angle > Math.toRadians(45)) {
            actionQueue.schedule(new SpecSlamCommand(deposit, lift, pivot));
        } else if (gamepad2.dpad_left && !isPr && pivot.angle < Math.toRadians(45)){
            actionQueue.schedule(new PickAndFlipCommand(deposit, lift, pivot));
        } isPr = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !irp) {
            if (lift.ticks > 500) {
                lift.setTarget(0);
            } else {
                lift.setTarget(Lift.limit);
            }
        } irp = gamepad2.dpad_right;

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("Inverted Controls?", g);
        telemetry.addData("Hanging", hang);
        telemetry.update();
    }
}
