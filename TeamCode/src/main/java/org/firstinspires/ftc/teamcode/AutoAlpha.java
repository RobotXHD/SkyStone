package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoAlpha extends LinearOpMode {
    HardwareSkybot r = new HardwareSkybot();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init();
    }
}
