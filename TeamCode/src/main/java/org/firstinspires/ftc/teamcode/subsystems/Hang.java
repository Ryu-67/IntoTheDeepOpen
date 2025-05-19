package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Hang {

    private CRServo hang1, hang2;

    public static double holdPower = 0, winchIn = 1, deploy = -1;
    public static DcMotorSimple.Direction hang1Direction = DcMotorSimple.Direction.FORWARD,
            hang2Direction = DcMotorSimple.Direction.REVERSE;

    public Hang(HardwareMap hardwareMap) {
        hang1 = hardwareMap.crservo.get("hang1");
        hang2 = hardwareMap.crservo.get("hang2");

        hang1.setDirection(hang1Direction);
        hang2.setDirection(hang2Direction);
    }

    public void setPower(double power) {
        hang1.setPower(power);
        hang2.setPower(power);
    }

    private double lastPower = 0;

    public void update(boolean in, boolean out) {
        if (!in && !out && lastPower != holdPower) {
            setPower(holdPower);
        } else if (in && !out && lastPower != winchIn) {
            setPower(winchIn);
        } else if (!in && out && lastPower != deploy) {
            setPower(deploy);
        }

        lastPower = hang1.getPower();
    }

}
