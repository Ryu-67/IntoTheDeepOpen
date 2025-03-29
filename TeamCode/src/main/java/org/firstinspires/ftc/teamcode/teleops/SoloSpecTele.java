package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.cmd.base.PickInCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.base.PickoutCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.GoToSlamTeleCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.spec.UnSlamTeleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Specimen Tele")
@Disabled
public class SoloSpecTele extends OpMode {

    public Drive drive; public MPPivot pivot; public Deposit deposit; public Lift lift;
    public Robot robot;

    CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true, true, true);
        drive = new Drive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        pivot = robot.pivot;
        deposit = robot.deposit;
        lift = robot.lift;

        pivot.setTargetAngle(0);

        actionQueue.reset();
        actionQueue = CommandScheduler.getInstance();

        actionQueue.reset();
    }

    boolean hang = false; boolean irp = false, ip2 = false, ip3 = false, g = false, isPr = false, ih = false, hangOn = false;
    double m = 1;

    @Override
    public void loop() {

        drive.setXYZ(gamepad1.left_stick_x * m, gamepad1.left_stick_y * m, gamepad1.right_stick_x * m);
        lift.setManualInput(gamepad1.right_trigger-gamepad1.left_trigger);

        if (!ip2 && gamepad1.x) {
            pivot.setTargetAngle(MainArm.vertAngle);
            lift.setLimit(true);
        } else if (!ip2 && gamepad1.y) {
            lift.setLimit(true);
            pivot.setTargetAngle(MainArm.hangle);
        } else if (!ip2 && gamepad1.b) {
            lift.setLimit(false);
            pivot.setTargetAngle(0);
        }
        ip2 = gamepad1.b || gamepad1.a || gamepad1.x;
        deposit.process(gamepad1.dpad_up, gamepad1.dpad_down);

        actionQueue.run();

        if (gamepad1.a && !ih && lift.ticks > 500) {
            actionQueue.schedule(new PickoutCommand(deposit, lift));
        } else if (gamepad1.a && ! ih && lift.ticks < 500) {
            actionQueue.schedule(new PickInCommand(deposit, lift));
        } ih = gamepad1.a;

        if (gamepad1.dpad_left && !isPr && pivot.angle > Math.toRadians(60)) {
            actionQueue.schedule(new GoToSlamTeleCommand(deposit, lift, pivot));
        } else if (gamepad1.dpad_left && !isPr && pivot.angle < Math.toRadians(60)){
            actionQueue.schedule(new UnSlamTeleCommand(pivot, lift, deposit));
        } isPr = gamepad2.dpad_left;

        if (gamepad1.dpad_right && !irp) {
            if (lift.ticks > 500) {
                lift.setTarget(0);
            } else {
                lift.setTarget(Lift.limit);
            }
        } irp = gamepad1.dpad_right;

        lift.update();
        pivot.update();
        drive.update();
        deposit.update(gamepad1.left_bumper, gamepad1.right_bumper);

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("Inverted Controls?", g);
        telemetry.addData("Hanging", hangOn);
        telemetry.update();
    }
}
