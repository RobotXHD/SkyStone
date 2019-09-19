package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

@TeleOp
public class TeleOp_Colect extends OpMode {
    /**declare the motors*/
    private DcMotorEx motordf;
    private DcMotorEx motorsf;
    private DcMotorEx motords;
    private DcMotorEx motorss;
    private DcMotor  motorColectSt, motorColectDr;
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
    private boolean stop;
    /**variables that count the thread's fps*/
    private long fpsC=0;
    private long fpsCLast;
    /** variable that  holds the system current time milliseconds */
    private long sysTimeC,sysTimeP;
    /** variables that toggle motors collect */
    private boolean apoz = false, alast = true, apoz2 = false, alast2 = true;
    private double power = 1;

    private Thread Chassis_Colect = new Thread( new Runnable() {
        @Override
        public void run() {
            /**repeat until the program stops*/
            while (!stop) {
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

                /**calculating the power for motors*/
                df = forward + clockwise - rright;
                ss = forward - clockwise - rright;
                sf = -forward + clockwise - rright;
                ds = -forward - clockwise - rright;

                /**normalising the power values*/
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

                /** fps counter */
                fpsC++;
                if (sysTimeC + 1000 < System.currentTimeMillis()) {
                    fpsCLast = fpsC;
                    fpsC = 0;
                    sysTimeC = System.currentTimeMillis();
                }

                /** setting the speed of the chassis*/
                if (v == 1) {
                    POWER(df / 5, sf / 5, ds / 5, ss / 5);
                } else if (v == 2) {
                    POWER(df, sf, ds, ss);
                }

                if(gamepad1.dpad_up && sysTimeP +200 < System.currentTimeMillis() && power < 1) {
                    power += 0.01;
                    sysTimeP = System.currentTimeMillis();
                    motorColectSt.setPower(power);
                    motorColectDr.setPower(-power);
                }
                if(gamepad1.dpad_down && sysTimeP +200 < System.currentTimeMillis() && power > 0) {
                    power -= 0.01;
                    sysTimeP = System.currentTimeMillis();
                    motorColectSt.setPower(power);
                    motorColectDr.setPower(-power);
                }
                /**set the collector motors on or off using the toggle*/
                boolean abut = gamepad1.x;
                if(alast != abut){
                    if(gamepad1.x) {
                        apoz = !apoz;
                        if (apoz){
                            motorColectSt.setPower(power);
                            motorColectDr.setPower(-power);
                        }
                        else{
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast=abut;
                }

                boolean abut2 = gamepad1.y;
                if(alast2 != abut2){
                    if(gamepad1.y) {
                        apoz2 = !apoz2;
                        if (apoz2){
                            motorColectSt.setPower(-1);
                            motorColectDr.setPower(1);
                        }
                        else{
                            motorColectSt.setPower(0);
                            motorColectDr.setPower(0);
                        }
                    }
                    alast2=abut2;
                }

            }
        }
    });


    @Override
    public void init(){
        /**initialization motors*/
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");

        motorColectDr = hardwareMap.get(DcMotor.class, "encoderDreapta");
        motorColectSt = hardwareMap.get(DcMotor.class, "encoderSpate");

        motords.setDirection(DcMotorSimple.Direction.REVERSE);
        motorss.setDirection(DcMotorSimple.Direction.REVERSE);

        /**set the mode of the motors*/
        motordf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**initialization system current milliseconds */
        sysTimeC = System.currentTimeMillis();
        sysTimeP = sysTimeC;

        /**start the thread*/
        Chassis_Colect.start();

    }

    /**using the loop function to send the telemetry to the phone*/
    @Override
    public void loop(){
        telemetry.addData("motordf: ", motordf.getCurrentPosition());
        telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
        telemetry.addData("motords: ", motords.getCurrentPosition());
        telemetry.addData("motorss: ", motorss.getCurrentPosition());
        telemetry.addData("Th: ", fpsCLast);
        telemetry.addData("POWER: ", power);
        telemetry.update();
    }

    /**using the stop function to stop the threads */
    public void stop(){stop = true;}

    /**the power function sets the motor's power*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motordf.setPower(df1);
        motorss.setPower(ss1);
        motorsf.setPower(sf1);
        motords.setPower(ds1);
    }

}
