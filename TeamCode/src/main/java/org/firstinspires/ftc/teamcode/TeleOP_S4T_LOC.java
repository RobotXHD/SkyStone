package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

@TeleOp
public class  TeleOP_S4T_LOC extends OpMode {
    /***declare the motors and encoders */
    private DcMotor encoderDreapta , encoderSpate, encoderStanga;
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    /**variable for changing the movement speed of the robot*/
    private int v = 2;
    /**variables for calculating the power for motors*/
    private double df;
    private double sf;
    private double ds;
    private double ss;
    private double max;
    /**variables for holding the gamepad joystick values;
     * we don't want to access them too many times in a loop */
    private double forward, rright, clockwise;
    /**variable that stops the threads when programs stop*/
    private boolean stop = false;
    /**variables that count the thread's fps*/
    private long fpsC=0, fpsEncDr = 0, fpsEncSp = 0, fpsEncSt = 0, fpsLOC = 0;
    private long fpsCLast, fpsEncSpLast, fpsEncDrLast, fpsEncStLast, fpsLOCLast;
    /** variables that  hold the system current time milliseconds*/
    private long sysTimeC, sysTimeEncSp, sysTimeEncDr, sysTimeEncSt, sysTimeLOC, sysTimeP, sysTimeI, sysTime;
    /**variables that holds the encoders value*/
    private int EncDr, EncSp, EncSt;
    /**variables for calculating the pid */
    private double angle, fs, sideways;
    private double d = 377, omniLenght = 188.49555921538759430775860299677, rotationCircleLenght = PI * d, tickPerDeg = (rotationCircleLenght / omniLenght / 360) * 4000;
    private double encRot, targetAngle, delta, p, i, kp = -0.16, ki = -0.3, kd, corr;
    private boolean rot = true;


    private Thread ENCDr = new Thread(new Runnable() {
        int encdr;
        @Override
        public void run() {
            /**repeat until the program stops*/
            while(!stop){
                /**reading the encoder*/
                encdr = encoderDreapta.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncDr = encdr;

                /**fps counter*/
                fpsEncDr++;
                if (sysTimeEncDr + 3000 < System.currentTimeMillis()) {
                    fpsEncDrLast = fpsEncDr / 3;
                    fpsEncDr = 0;
                    sysTimeEncDr = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCSp = new Thread(new Runnable() {
        int encsp;
        @Override
        public void run() {
            /**repeat until the program stop*/
            while(!stop){
                /**reading the encoders*/
                encsp = encoderSpate.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncSp = encsp;

                /**fps counter*/
                fpsEncSp++;
                if (sysTimeEncSp + 3000 < System.currentTimeMillis()) {
                    fpsEncSpLast = fpsEncSp/3;
                    fpsEncSp = 0;
                    sysTimeEncSp = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread ENCSt = new Thread(new Runnable() {
        int encst;
        @Override
        public void run() {
            /**repeat until the program stop*/
            while(!stop){
                /**reading the encoders*/
                encst = encoderStanga.getCurrentPosition();
                /**assigning the encoders value to another global value*/
                EncSt = -encst;

                /**fps counter*/
                fpsEncSt++;
                if(sysTimeEncSt + 3000 < System.currentTimeMillis()){
                    fpsEncStLast = fpsEncSt/3;
                    fpsEncSt = 0;
                    sysTimeEncSt = System.currentTimeMillis();
                }
            }
        }
    });

    private Thread Chassis = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
                /**calibrating the ki, kp*/
                if(gamepad1.dpad_down && sysTimeI + 200 < System.currentTimeMillis()){
                    sysTimeI = System.currentTimeMillis();
                    ki-=0.01;
                }
                else if(gamepad1.dpad_up && sysTimeI + 200 < System.currentTimeMillis()){
                    sysTimeI = System.currentTimeMillis();
                    ki+=0.01;
                }

                if(gamepad1.dpad_right && sysTimeP + 200 < System.currentTimeMillis()){
                    sysTimeP = System.currentTimeMillis();
                    kp+=0.01;
                }
                else if(gamepad1.dpad_left && sysTimeP + 200 < System.currentTimeMillis()){
                    sysTimeP = System.currentTimeMillis();
                    kp-=0.01;
                }


                if(gamepad1.b) {
                    double targetAngle = 0;
                    if(angle < targetAngle){
                        while(angle < targetAngle - 3 && !stop){
                            POWER(-0.1, -0.1, 0.1, 0.1);
                        }
                    }
                    else if(angle > targetAngle){
                        while(angle > targetAngle + 3 && !stop){
                            POWER(0.1, 0.1, -0.1, -0.1);
                        }
                    }
                    POWER(0,0,0,0);
                }
                else {
                    /**change the variable that controls the speed of the chassis using the bumpers*/
                    if (gamepad1.right_bumper) {
                        v = 1;
                    } else if (gamepad1.left_bumper) {
                        v = 2;
                    }
                    /**getting the gamepad joystick values*/
                    forward = gamepad1.left_stick_y;
                    rright = -gamepad1.left_stick_x;
                    clockwise = gamepad1.right_stick_x;

                    /**calculating the power for the motors*/
                    df = forward + clockwise - rright;
                    ss = forward - clockwise - rright;
                    sf = -forward + clockwise - rright;
                    ds = -forward - clockwise - rright;

                    if(clockwise == 0){
                        if(rot){
                            targetAngle = angle;
                            rot = false;
                        }

                        delta = (targetAngle - Math.round(angle))/10;
                        if(delta > 1 || delta < -1){
                            delta = delta / Math.abs(delta);
                        }
                        p = delta * kp;

                        i = Math.signum(delta)*(delta * delta)/2 * ki;

                        kd = 0;
                        corr = p + i + kd;

                        sf += corr;
                        ss -= corr;
                        df += corr;
                        ds -= corr;

                    }
                    else{
                        rot = true;
                    }

                    /**normalising the power values */
                    max = abs(sf);
                    if (abs(df) > max) {
                        max = abs(df);
                    }
                    if (abs(ss) > max) {
                        max = abs(ss);
                    }
                    if (abs(ds) > max) {
                        max = abs(ds);

                    }
                    if (max > 1) {
                        sf /= max;
                        df /= max;
                        ss /= max;
                        ds /= max;
                    }

                    /**setting the speed of the chassis*/
                    if (v == 1) {
                        POWER(df / 5, sf / 5, ds / 5, ss / 5);
                    } else if (v == 2) {
                        POWER(df, sf, ds, ss);
                    }

                    /**fps counter*/
                    fpsC++;
                    if (sysTimeC + 1000 < System.currentTimeMillis()) {
                        fpsCLast = fpsC;
                        fpsC = 0;
                        sysTimeC = System.currentTimeMillis();
                    }
                }
            }
        }
    });

    private Thread LOC = new Thread(new Runnable() {
        double offset = 0;
        @Override
        public void run() {
            /**repeat until the program stop*/
            while (!stop) {
                if(gamepad1.y){
                    offset = (EncDr - EncSt) / 2.0;
                }
                encRot = ((EncDr - EncSt) / 2.0) - offset;
                angle = encRot / tickPerDeg;

                /**fps counter*/
                fpsLOC++;
                if (sysTimeLOC + 3000 < System.currentTimeMillis()) {
                    fpsLOCLast = fpsLOC/3;
                    fpsLOC = 0;
                    sysTimeLOC = System.currentTimeMillis();
                }
            }
        }
    });

    @Override
    public void init() {

        /**initialization motor and the encoders */
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        encoderDreapta = hardwareMap.get(DcMotor.class,"encoderDreapta");
        encoderSpate = hardwareMap.get(DcMotor.class,"encoderSpate");
        encoderStanga = hardwareMap.get(DcMotor.class, "encoderStanga");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);


        /**setting the mode of the motors and the mode of the encoders*/
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization the system current time milliseconds*/
        sysTime = System.currentTimeMillis();
        sysTimeC = sysTime;
        sysTimeEncDr = sysTime;
        sysTimeEncSp = sysTime;
        sysTimeEncSt = sysTime;
        sysTimeLOC = sysTime;
        sysTimeP = sysTime;
        sysTimeI = sysTime;

        /**starting the threads*/
        ENCDr.start();
        ENCSp.start();
        ENCSt.start();
        Chassis.start();
        LOC.start();

    }

    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop() {
        telemetry.addData("ENCDr:", EncDr);
        telemetry.addData("ENCSp:", EncSp);
        telemetry.addData("ENCSt:", EncSt);
        telemetry.addData("EncRot", encRot);

        telemetry.addData("Angle:", angle);

        telemetry.addData("motordf: ", df);
        telemetry.addData("motorsf: ", sf);
        telemetry.addData("motords: ", ds);
        telemetry.addData("motorss: ", ss);

        telemetry.addData("p: ", p);
        telemetry.addData("i: ", i);
        telemetry.addData("kp: ", kp);
        telemetry.addData("ki: ", ki);

        /*telemetry.addData("Th Chassis: ", fpsCLast);
        telemetry.addData("Th EncSp: ", fpsEncSpLast);
        telemetry.addData("Th EncDr: ", fpsEncDrLast);
        telemetry.addData("Th EncSt: ", fpsEncStLast);
        telemetry.addData("Th LOC: ", fpsLOCLast);*/
        telemetry.update();
    }

    /**using the stop function to stop the threads*/
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }
}