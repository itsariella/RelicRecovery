
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

@TeleOp(name="CompetitionBot2", group="Pushbot")
public class CompetitionBotAutomated extends OpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor lift;
    public DcMotor intakeRight;
    public DcMotor intakeLeft;

    public Servo s1;
    public Servo s2;
    public Servo arm;
    public Servo catcherLeft;
    public Servo catcherRight;

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
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        arm = hardwareMap.servo.get("arm");
        catcherLeft = hardwareMap.servo.get("catcherLeft");
        catcherRight = hardwareMap.servo.get("catcherRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);


        s2.setDirection(Servo.Direction.REVERSE);
        catcherRight.setDirection(Servo.Direction.REVERSE);

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
        double liftPower = 0;
        double intakeRightPower;
        double intakeLeftPower;

        // Gamepad 1 controls
        if (Math.abs(gamepad1.left_stick_x) > .1)
            x = gamepad1.left_stick_x;
        else
            x = 0;

        if (Math.abs(gamepad1.left_stick_y) > .1)
            y = gamepad1.left_stick_y;
        else
            y = 0;

        if (Math.abs(gamepad1.right_stick_x) > .1)
            z = gamepad1.right_stick_x;
        else
            z = 0;

        if (gamepad1.right_trigger > .1) {
            intakeRightPower = gamepad1.right_trigger;
        } else if (gamepad1.right_bumper) {
            intakeRightPower = -0.7;
        } else {
            intakeRightPower = 0;
        }

        if (gamepad1.left_trigger > .1) {
            intakeLeftPower = gamepad1.left_trigger;
            catcherLeft.setPosition(1);
            catcherRight.setPosition(1); // use catchers
            s1.setPosition(1);
            s2.setPosition(1); //open arms when taking in glyphs
        }
        else if (gamepad1.left_bumper)
        {
            intakeLeftPower = -0.7; // reverse blocks
        }
        else {
            intakeLeftPower = 0;
        }

        if(gamepad1.a) {
            arm.setPosition(1); // jewel arm
        }

        if(gamepad1.b){
            arm.setPosition(0);
        }

        // Gamepad 2 controls
        if (Math.abs(gamepad2.left_stick_y) > .1 ) {
            liftPower = gamepad2.left_stick_y; // set power to lift
        }

        if (gamepad2.right_trigger > 0.1) {
            s1.setPosition(.5);
            s2.setPosition(.5); // close arms
        }

        if (gamepad2.left_trigger > 0.1) {
            s1.setPosition(1);
            s2.setPosition(1); // open arms
        }

        if (gamepad2.right_bumper) {
            catcherLeft.setPosition(1); // use catchers
            catcherRight.setPosition(1);
        }
        if (gamepad1.left_bumper) {
            catcherLeft.setPosition(0); // move away catchers
            catcherRight.setPosition(0);
        }

        if (gamepad2.x) { //GRABBING ONE BLOCK
            s1.setPosition(.5); // close arms to grab block
            s2.setPosition(.5);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoders
            lift.setTargetPosition(1120); // carries glyph to position 1600
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.5); // half speed
        }

        if (gamepad2.y) { //RETRIEVE TWO BLOCKS
            s1.setPosition(1); // let go of glyphs
            s2.setPosition(1);
            lift.setTargetPosition(0); // maybe reverse to starting position
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.5);
            s1.setPosition(0.5); //grab glyphs
            s2.setPosition(0.5);
            lift.setTargetPosition(750);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.5);
            catcherLeft.setPosition(0);
            catcherRight.setPosition(0);
        }

        if (gamepad2.b) { //RELEASE BLOCKS
            catcherLeft.setPosition(0);
            catcherRight.setPosition(0);
            s1.setPosition(1);
            s2.setPosition(1);
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.5);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        // Set powers
        lift.setPower(liftPower);
        intakeLeft.setPower(-intakeLeftPower);
        intakeRight.setPower(-intakeRightPower);
        frontLeft.setPower(y+x+z);
        backLeft.setPower(y-x+z);
        frontRight.setPower(y-x-z);
        backRight.setPower(y+x-z);
}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
