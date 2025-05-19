package org.firstinspires.ftc.teamcode.autos.cmd;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.test.PNPTest;

public class StartStream extends CommandBase {
    CameraSensor sensor;
    boolean enabled = false;

    public StartStream(CameraSensor sensor, boolean enabled) {
        this.sensor = sensor;
        this.enabled = enabled;
    }

    @Override
    public boolean isFinished() {
        sensor.setEnabled(enabled);

        return true;
    }
}
