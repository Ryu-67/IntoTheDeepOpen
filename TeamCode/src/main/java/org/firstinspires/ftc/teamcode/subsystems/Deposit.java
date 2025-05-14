package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Deposit {

    private Servo lArm, pitch, wrist;
    private Claw claw;

    public static double vert = 0.5, hori = 0.03, wrapped = 0.98, diag = 0.48/2;

    private boolean ispressed = false, spec = false;

    public static double aPitchUp=0.455, aPitchDown = 0.84, aPitchBack = 0.07
            , aPitchSpec = 0.44, specSlamPitch = 0.6, slamOnPitch = 0.45;

    public static double armDown = 0.23, armUp = 0.27, armBack = 0.23, armSpec = 0.48, specSlam = 0.4, slamPos = 0.15;

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

        claw.update(clawT);

        if (wristT && !ispressed) {
            if (wstate == Wstate.hori) {
                wrist.setPosition(vert);
                wstate = Wstate.vert;
            } else {
                wrist.setPosition(hori);
                wstate = Wstate.hori;
            }
        } ispressed = wristT;

        if (incr != lincr) {
            if (incr == 1) {
                    if (spec) {
                        setArm(armSpec);
                        pitch.setPosition(aPitchSpec);
                    } else {
                        setArm(armUp);
                        pitch.setPosition(aPitchUp);
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

    public void setClaw(double clawP) {
        claw.directSet(clawP);
    }

    public double getArm() {
        return lArm.getPosition();
    }

}
