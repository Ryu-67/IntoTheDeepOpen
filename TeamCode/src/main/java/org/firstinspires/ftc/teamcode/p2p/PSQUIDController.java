package org.firstinspires.ftc.teamcode.p2p;

public class PSQUIDController {

    private double p, error, damper = 1, sign = 1;

    public PSQUIDController(double p) {
        this.p = p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double compute(double error) {
        sign = Math.signum(error);
        return p*Math.sqrt(error)*damper*sign;
    }

    public void setDamper(double damper) {
        this.damper = damper;
    }

}
