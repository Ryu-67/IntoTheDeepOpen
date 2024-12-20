package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Deposit {

    private Servo lArm, pitch, wrist;
    private Claw claw;

    public static double vert = 0.48, hori = 0, wrapped = 0.95;

    private boolean ispressed = false, spec = false;

    public static double aPitchUp=0.7, aPitchDown = 1, aPitchBack = 0.06, aPitchSpec = 0.8, specSlamPitch = 0.6;

    public static double armDown = 1, armUp = 0.76, armBack = 0.63, armSpec = 1, specSlam = 0.5;
    public Deposit(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("arm");
        pitch = hardwareMap.servo.get("pitch");
        pitch.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");
        claw = new Claw(hardwareMap);

        claw.directSet(Claw.closed);
    }

    public Deposit(HardwareMap hardwareMap, boolean spec) {
        lArm = hardwareMap.servo.get("arm");
        pitch = hardwareMap.servo.get("pitch");
        pitch.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");
        claw = new Claw(hardwareMap);

        claw.directSet(Claw.closed);

        this.spec = spec;
    }

    public int incr = 1; boolean lock = false; int lincr = 0;


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

    private enum Wstate {
        hori, vert, wrapped
    }

    Wstate wstate = Wstate.hori;

    public void update(boolean wristT, boolean clawT) {
        if (wristT && !ispressed) {
            if (wstate == Wstate.hori) {
                wrist.setPosition(vert);
                wstate = Wstate.vert;
            } else if (wstate == Wstate.vert) {
                wrist.setPosition(wrapped);
                wstate = Wstate.wrapped;
            } else {
                wrist.setPosition(hori);
                wstate = Wstate.hori;
            }
        } ispressed = wristT;

        claw.update(clawT);

        if (incr != lincr) {
            if (incr == 1) {
                if (!spec) {
                    setArm(armUp);
                    pitch.setPosition(aPitchUp);
                } else {
                    setArm(armSpec);
                    pitch.setPosition(aPitchSpec);
                }
            } else if (incr == 0) {
                setArm(armDown);
                pitch.setPosition(aPitchDown);
            } else {
                setArm(armBack);
                pitch.setPosition(aPitchBack);
            }
            lincr = incr;
        }
    }

    public static double degToRange(double deg) {
        return Range.clip(deg, 0, 300)/300;
    }

    public void setArm(double val) {
        lArm.setPosition(val);
    }

    public void setDeposit(double wristp, double pitchp, double armp, double clawp) {
        wrist.setPosition(wristp); setArm(armp); claw.directSet(clawp); pitch.setPosition(pitchp);
    }

    public double wristP() {
        return wrist.getPosition();
    }

    public double readClaw(){
        return claw.gCLaw();
    }

}
