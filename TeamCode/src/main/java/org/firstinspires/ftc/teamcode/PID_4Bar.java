package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class PID_4Bar extends OpMode {
    private DcMotorEx motorBratStanga;
    boolean stop = false, targetSet = false;
    long target, position, encoderStanga, dt = 200;
    double sysTime;
    double kp = 0, ki = 0, kd = 0;
    double p = 0 , i = 0 , d = 0 ;
    @Override
    public void init() {
        motorBratStanga = hardwareMap.get(DcMotorEx.class,"bratStanga");

        motorBratStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorBratStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        sysTime = System.currentTimeMillis();

        brat.start();
        EncoderStanga.start();

    }
    private Thread brat = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop) {
                if (gamepad1.x) {
                    if (targetSet) {
                        targetSet = false;
                    }
                    motorBratStanga.setPower(-0.5);

                } else if (gamepad1.b) {
                    if (targetSet) {
                        targetSet = false;
                    }
                    motorBratStanga.setPower(0.5);
                } else {
                    if (!targetSet) {
                        target = encoderStanga;
                        targetSet = true;
                    }
                    motorBratStanga.setPower(PID(encoderStanga - target, kp, ki, kd));
                }
                if (gamepad1.dpad_up && sysTime + dt < System.currentTimeMillis()) {
                    kp += 0.0001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.dpad_down && sysTime + dt < System.currentTimeMillis()) {
                    kp -= 0.0001;
                    sysTime = System.currentTimeMillis();
                }
                if (gamepad1.dpad_left && sysTime + dt < System.currentTimeMillis()) {
                    ki += 0.000001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.dpad_right && sysTime + dt < System.currentTimeMillis()) {
                    ki -= 0.000001;
                    sysTime = System.currentTimeMillis();
                }
                if (gamepad1.right_bumper && sysTime + 500 < System.currentTimeMillis()) {
                    kd += 0.0001;
                    sysTime = System.currentTimeMillis();
                } else if (gamepad1.left_bumper && sysTime + 500 < System.currentTimeMillis()) {
                    kd -= 0.0001;
                    sysTime = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread EncoderStanga = new Thread(new Runnable() {
        long es;
        @Override
        public void run() {
            while(!stop){
                es = motorBratStanga.getCurrentPosition();
                encoderStanga = es; // anti locking
            }
        }
    });
    @Override
    public void stop(){
        stop = true;
    }
    @Override
    public void loop() {
        telemetry.addData("kP", kp);
        telemetry.addData("kI", ki);
        telemetry.addData("kD", kd);
        telemetry.addData("target", target);
        telemetry.addData("position", encoderStanga);
        telemetry.update();
    }
    private double PID (double delta, double kp, double ki, double kd){
        p = delta * kp; // se calibreaza prima prin kp
        i = i + delta; //se calibreaza a doua prin ki, aduna eroarea in timp (cu cat e mai mult timp eroarea, cu atat corectia e mai mare)
        d = kd; // se calibreaza a 3 a, derivata lui f(x) = kd * x este kd
        return (p + i * ki + d);
    }
}
