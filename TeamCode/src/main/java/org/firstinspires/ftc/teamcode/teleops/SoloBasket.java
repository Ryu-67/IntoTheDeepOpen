package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.autos.cmd.basket.DropAndResetCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.FlipAndExtendCommand;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.GoToPreIntake;
import org.firstinspires.ftc.teamcode.autos.cmd.basket.Grasp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "One Driver Basket Tele")
public class SoloBasket extends OpMode {

    public Drive drive; public MPPivot pivot; public Deposit deposit; public Lift lift;
    public PinpointLocalizer localizer;

    public Robot robot;



    public CommandScheduler actionQueue = CommandScheduler.getInstance();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true, false, true);
        drive = robot.drive;
        pivot = robot.pivot;
        lift = robot.lift;
        deposit = new Deposit(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap, new Pose(0, 0, Math.toRadians(-90)));

        pivot.setTargetAngle(0);
        lift.setTarget(0);

        actionQueue.reset();
        actionQueue = CommandScheduler.getInstance();
    }
    boolean ip2 = false, isPr2 = false, irp = false, intakeState = false, aPress = false;

    @Override
    public void loop() {
        drive.setXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        deposit.process(gamepad1.dpad_up, gamepad1.dpad_down);
        lift.setManualInput(gamepad1.right_trigger-gamepad1.left_trigger);

        if (!ip2 && gamepad1.b) {
            pivot.setTargetAngle(0);
            lift.setLimit(false);
        } else if (!ip2 && gamepad1.x) {
            pivot.setTargetAngle(86);
            lift.setLimit(true);
        }
        ip2 = gamepad1.b|| gamepad1.x;

        actionQueue.run();

        drive.update();
        lift.update();
        pivot.update();
        localizer.update();
        deposit.update(gamepad1.left_bumper, gamepad1.right_bumper);

        if (gamepad1.dpad_left && !isPr2 && Math.toDegrees(pivot.angle) > MainArm.hangle) {
            actionQueue.schedule(new DropAndResetCommand(deposit, lift, pivot));
            deposit.incr = 1;
        } isPr2 = gamepad1.dpad_left;


        if (pivot.angle > Math.toRadians(35)) {
            lift.setLimit(true);
            lift.setPID(0.016);
        } else {
            lift.setLimit(false);
            lift.setPID(0.01);
        }

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("slide reset switch", lift.readSensor());
        telemetry.addData("Lift Ticks", lift.getTicks());
        telemetry.addData("Lift Target", lift.target);
        telemetry.update();
    }
}