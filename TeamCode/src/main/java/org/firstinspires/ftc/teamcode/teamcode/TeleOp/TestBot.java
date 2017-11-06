
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Testbot", group="Pushbot")
public class TestBot extends OpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    //Servos
    public Servo leftArm;
    public Servo rightArm;

    public String hello = "hello";


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(Servo.Direction.REVERSE);

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

        double frMotor = 0;
        double brMotor = 0;
        double flMotor = 0;
        double blMotor = 0;
        double leftY;
        double leftX;
        double rightX;

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;

        if (Math.abs(gamepad1.left_stick_y) < .1) {
            leftY = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) < .1) {
            leftX = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) < .1) {
            rightX = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1) {
            frMotor = (leftX - leftY) * .4;
            flMotor = (-leftX - leftY) * .4;
            brMotor = (-leftX - leftY) * .4;
            blMotor = (leftX - leftY) * .4;
        }

        if (Math.abs(gamepad1.right_stick_x) > .1){
            frMotor = (rightX) * .4;
            flMotor = -(rightX) * .4;
            brMotor = (rightX) * .4;
            blMotor = -(rightX) * .4;
        }

        if (gamepad1.left_bumper) {
            leftArm.setPosition(.4);
            rightArm.setPosition(.4);
        }
        if (gamepad1.right_bumper){
            leftArm.setPosition(.1);
            rightArm.setPosition(.1);
        }

        frontRight.setPower(frMotor);
        backRight.setPower(brMotor);
        frontLeft.setPower(flMotor);
        backLeft.setPower(blMotor);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
