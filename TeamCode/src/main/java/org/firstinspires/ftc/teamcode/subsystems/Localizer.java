package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Localizer {

    private SparkFunOTOS otos;
    public static Pose2D offset = new SparkFunOTOS.Pose2D(2.5, 0, 90);

    public Localizer(HardwareMap hardwareMap) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
    }

    public void setPose(Pose2D pose) {
        otos.setPosition(pose);
    }

    public Pose2D getPose() {
        return otos.getPosition();
    }

    private void configureOtos() {

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();

        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

}
