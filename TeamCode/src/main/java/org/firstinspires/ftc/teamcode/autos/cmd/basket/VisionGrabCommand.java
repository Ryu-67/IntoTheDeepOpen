package org.firstinspires.ftc.teamcode.autos.cmd.basket;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ocv.SolvePNP;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MPPivot;

public class VisionGrabCommand extends CommandBase {

    CameraSensor sensor; SolvePNP solvePNP;
    Deposit deposit; MPPivot pivot; PinpointDrive drive;

    double wrist = 0; boolean rLoop = true; ElapsedTime elapsedTime = new ElapsedTime();

    private enum State {
        scanning,
        shifting,
        calculating,
        grabbing
    }

    State state = State.scanning;

    public VisionGrabCommand(CameraSensor sensor, Deposit deposit, MPPivot pivot, PinpointDrive drive,
                             double wrist) {
        this.sensor = sensor; this.deposit = deposit; this.pivot = pivot; this.drive = drive;
        this.wrist = wrist; solvePNP = new SolvePNP();
    }

    @Override
    public void execute() {
        if (rLoop) {
            rLoop = false;
            elapsedTime.reset();
        }

        switch (state) {

        }
    }

}
