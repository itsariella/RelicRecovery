// This is the OpMode we use during Tele-Op
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@TeleOp(name="CompetitionBot", group="Pushbot")
public class CompetitionBot extends OpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor lift;
    public DcMotor intakeRight;
    public DcMotor intakeLeft;


    public Servo firstStage1; //right?
    public Servo firstStage2; //left?
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

        firstStage1 = hardwareMap.servo.get("firstStage1");
        firstStage2 = hardwareMap.servo.get("firstStage2");
        arm = hardwareMap.servo.get("arm2");
        catcherLeft = hardwareMap.servo.get("catcherLeft");
        catcherRight = hardwareMap.servo.get("catcherRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        catcherLeft.setDirection(Servo.Direction.REVERSE);
        firstStage2.setDirection(Servo.Direction.REVERSE);

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

        if(gamepad1.right_trigger > .1 && gamepad1.left_trigger >.1){
            catcherLeft.setPosition(0);
            catcherRight.setPosition(0.01);
        }

        if (gamepad1.right_trigger > .1) {
            intakeRightPower = gamepad1.right_trigger;

        } else if (gamepad1.right_bumper) {
            intakeRightPower = -0.7;
        } else {
            intakeRightPower = 0;
        }

        if (gamepad1.left_trigger > .1) { // IF USING INTAKE
            intakeLeftPower = gamepad1.left_trigger;
        } else if (gamepad1.left_bumper) {
            intakeLeftPower = -0.7; // 70 percent speed
        } else {
            intakeLeftPower = 0;
        }

        if (gamepad1.x) {
            arm.setPosition(0.6);
        }

        if (gamepad1.y) {
            arm.setPosition(0);
        }

        if (gamepad1.a) {
            catcherLeft.setPosition(0); // use catchers
            catcherRight.setPosition(0.01);
        }
        if (gamepad1.b) {
            catcherLeft.setPosition(1); // move away catchers
            catcherRight.setPosition(1);
        }

        // Gamepad 2 controls
        if (Math.abs(gamepad2.left_stick_y) > .1) {
            liftPower = gamepad2.left_stick_y;
        }
        /*if (gamepad2.y) {
            setFullPosition = true;
        }
        if (gamepad2.x) {
            setFullPosition = false;
        }*/

        if (gamepad2.right_trigger > 0.1) {
            firstStage1.setPosition(1); // first stage glyph arms close
            firstStage2.setPosition(1);
        }

        if (gamepad2.left_trigger > 0.1) {
            firstStage1.setPosition(0.9);
            firstStage2.setPosition(0.9); // open glyph arms
        }
        if(gamepad2.left_bumper){
            firstStage1.setPosition(.7); //glyph arms open
            firstStage2.setPosition(.7);
        }

        if (gamepad2.dpad_up) {
            catcherLeft.setPosition(0); // use catchers
            catcherRight.setPosition(0.01);
        }
        if (gamepad2.dpad_down) {
            catcherLeft.setPosition(1); // move away catchers
            catcherRight.setPosition(1);
        }

        if (gamepad2.a){

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //target position
            lift.setTargetPosition(-2750);

            //set mode
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //set power
            lift.setPower(1);

            if(lift.isBusy()){ //used to be while loop
                telemetry.addData("Path2",  "Running at %7d", lift.getCurrentPosition());
                telemetry.update();

            }
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set powers
            lift.setPower(liftPower);
            intakeLeft.setPower(-intakeLeftPower * .70);
            intakeRight.setPower(-intakeRightPower * .70);
            frontLeft.setPower(y + x + z);
            backLeft.setPower(y - x + z);
            frontRight.setPower(y - x - z);
            backRight.setPower(y + x - z);
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
        @Override
        public void stop () {
        }

    }

