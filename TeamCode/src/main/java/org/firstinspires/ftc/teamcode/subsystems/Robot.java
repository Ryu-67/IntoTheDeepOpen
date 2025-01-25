package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
public class Robot {

    public Lift lift; public MPPivot pivot; public Drive drive; public Deposit deposit;
    public VoltageSensor voltageSensor;
    public static double universalVoltage = 12.8;

    private boolean enableDrive = false, isTele = false;

    public Robot(HardwareMap hardwareMap, boolean isTele, boolean specimen, boolean enableDrive) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        if (enableDrive) {
            if (isTele) {
                drive = new Drive(hardwareMap, voltageSensor);
            }
        }
        this.enableDrive = enableDrive;
        this.isTele = isTele;

        this.lift = new Lift(hardwareMap, isTele, voltageSensor);
        this.pivot = new MPPivot(hardwareMap, voltageSensor);
        this.deposit = new Deposit(hardwareMap, specimen);
    }

    public void update() {
        if (enableDrive) {
            if (isTele) {
                drive.update();
            }
        }
        lift.update();
        pivot.update();
    }

    public void update(boolean f1, boolean f2) {
        if (enableDrive) {
            if (isTele) {
                drive.update();
            }
        }
        lift.update();
        pivot.update();
        deposit.update(f1, f2);
    }

}
