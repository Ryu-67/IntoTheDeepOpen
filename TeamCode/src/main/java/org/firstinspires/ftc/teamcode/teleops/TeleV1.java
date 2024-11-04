package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Macros;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

import javax.crypto.Mac;

@TeleOp(name = "Comp Tele")
public class TeleV1 extends OpMode {

    public Drive drive; public MainArm arm; public Deposit deposit;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        arm = new MainArm(hardwareMap, true);
        deposit = new Deposit(hardwareMap);

        arm.setMainState(MainArm.State.intake);
    }

    StateMachine retract;
    boolean fsmEnabled = false;

    @Override
    public void loop() {
        drive.setFlags(gamepad1.left_bumper, gamepad1.right_bumper);
        drive.setXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        deposit.process(gamepad2.dpad_up, gamepad2.dpad_down);


        if (rightTriggerBoolean() && !leftTriggerBoolean() && !fsmEnabled) {
            retract = Macros.backIn(arm, deposit, MainArm.State.intake);
            retract.start();
            incr = 0;
            fsmEnabled = true;
        } else if (leftTriggerBoolean() && !rightTriggerBoolean() && !fsmEnabled) {
            retract = Macros.backIn(arm, deposit, MainArm.State.basket);
            retract.start();
            incr = 2;
            fsmEnabled = true;
        }

        if (!fsmEnabled) {
            arm.setLmi(gamepad2.right_stick_x);
            increment(gamepad2.b, gamepad2.x);
        } else {
            arm.setLmi(0);
            retract.update();
            if (!retract.isRunning()) {
                fsmEnabled = false;
            }
            telemetry.addData("state", retract.getStateString());
        }

        drive.update();
        deposit.update(gamepad2.left_bumper, gamepad2.right_bumper);
        arm.update();

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("slide reset switch", arm.readLiftReset());
        telemetry.update();
    }

    int incr = 0, lastIncr = 0; boolean isPr = false;

    public void increment(boolean upFlag, boolean downFlag) {
        if (downFlag && !isPr && incr > 0) {
            incr -= 1;
            isPr = true;
        } else if (upFlag && !isPr && incr < 3) {
            incr += 1;
            isPr = true;
        } else if (!downFlag && !upFlag) {
            isPr = false;
        }

        if (incr == 1) {
            arm.setMainState(MainArm.State.inter);
        } else if (incr == 2) {
            arm.setMainState(MainArm.State.basket);
        } else if (incr == 3) {
            arm.setMainState(MainArm.State.backpickup);
        } else {
            arm.setMainState(MainArm.State.intake);
        }
    }

    public boolean leftTriggerBoolean() {
        return gamepad2.left_trigger > 0;
    }

    public boolean rightTriggerBoolean() {
        return gamepad2.right_trigger > 0;
    }
}
