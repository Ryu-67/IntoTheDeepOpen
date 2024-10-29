package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Arm {

    private Servo lArm, pitch, wrist;
    private Claw claw;

    public enum State {
        back, up, down
    }

    public static double vert = 0.4, hori = 0;

    private State state = State.back, lastState = State.up;

    private boolean ispressed = false, ispressed2 = false;

    public static double aDown = 50, aUp = 168, aBack = 188;
    public static double aPitchUp=0.5, aPitchDown = 0.2, aPitchBack = 1;

    private double armDown = degToRange(aDown), armUp = degToRange(aUp), armBack = degToRange(aBack);

    public Arm(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("arm");
        pitch = hardwareMap.servo.get("pitch");
        pitch.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");
        claw = new Claw(hardwareMap);

        setArm(armDown);
        pitch.setPosition(0.2);
        wrist.setPosition(hori);
    }

    int incr = 2; boolean lock = false; int lincr = 0;


    public void process(boolean down, boolean up) {
        if (down && !lock && incr > 0) {
            incr -= 1;
            lock = true;
        } else if (up && !lock && incr < 2) {
            incr += 1;
            lock = true;
        } else if (!down && !up) {
            lock = false;
        }
    }

    public void update(boolean wristT, boolean clawT) {
        if (wristT && !ispressed) {
            if (wrist.getPosition() == vert) {
                wrist.setPosition(hori);
            } else {
                wrist.setPosition(vert);
            }
        } ispressed = wristT;

        claw.update(clawT);

        if (incr != lincr) {
            if (incr == 1) {
                setArm(armUp);
                pitch.setPosition(0.5);
            } else if (incr == 0) {
                setArm(armDown);
                pitch.setPosition(0.2);
            } else {
                setArm(armBack);
                pitch.setPosition(1);
            }
            lincr = incr;
        }
    }

    private double degToRange(double deg) {
        return Range.clip(deg, 0, 300)/300;
    }

    public void setArm(double val) {
        lArm.setPosition(val);
    }

    public void setDeposit(double wristp, double pitchp, double armp, double clawp) {
        wrist.setPosition(wristp); pitch.setPosition(pitchp); lArm.setPosition(armp); claw.directSet(clawp);
    }

}
