package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
@Disabled
public class AutoAlpha extends LinearOpMode {
    HardwareSkybot r = new HardwareSkybot();
    @Override
    public void runOpMode() throws InterruptedException {
        r.init();

    }
}
