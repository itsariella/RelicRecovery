
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Pushbot: WFFTeleBot", group="Pushbot")
public class WFFTeleBot extends OpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor lift;
    public Servo s1;
    public Servo s2;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        lift = hardwareMap.dcMotor.get("lift");
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(Servo.Direction.REVERSE);

        s1.setPosition(0);
        s2.setPosition(0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float x;
        float y;
        float z;
        double liftPower;


        if(Math.abs(gamepad1.left_stick_x) > .1)
            x = gamepad1.left_stick_x;
        else
            x = 0;

        if(Math.abs(gamepad1.left_stick_y) > .1)
            y = gamepad1.left_stick_y;
        else
            y = 0;

        if(Math.abs(gamepad1.right_stick_x) > .1)
            z = gamepad1.right_stick_x;
        else
            z = 0;

        if(gamepad1.right_trigger > 0.1)
            liftPower = gamepad1.right_trigger;
        else if(gamepad1.left_trigger > 0.1)
            liftPower = -gamepad1.left_trigger;
        else
            liftPower = 0;

        if(gamepad1.right_bumper) {
                s1.setPosition(0);
                s2.setPosition(0);
        }

        if(gamepad1.left_bumper) {
                s1.setPosition(.5);
                s2.setPosition(.7);
            }

        /*if(grab == 1) {
            s1.setPosition(0);
            s2.setPosition(0);
        }
        if(grab == 0){
            s1.setPosition(.5);
            s2.setPosition(.7);
        }

        if(gamepad1.a)
            if(grab == 0)
                grab = 1;
            else if(grab == 1)
                grab = 0;

        if(grab == 0){
            s1.setPosition(0);
            s2.setPosition(0);
        }else if(grab ==1){
            s1.setPosition(.7);
            s2.setPosition(.7);
        } */


        lift.setPower(liftPower);
        frontRight.setPower((y+x+z)*.4);
        backRight.setPower((y-x+z)*.4);
        frontLeft.setPower((y-x-z)*.4);
        backLeft.setPower((y+x-z)*.4);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
