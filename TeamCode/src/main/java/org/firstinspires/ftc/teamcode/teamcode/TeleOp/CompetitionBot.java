
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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


    public Servo s1;
    public Servo s2;
    public Servo arm;

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


        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        arm = hardwareMap.servo.get("arm");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);


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
        double liftPower = 0;
        double intakeRightPower;
        double intakeLeftPower;


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

        /*if (gamepad2.right_trigger > 0.1)
            liftPower = gamepad2.right_trigger;
        else if (gamepad2.left_trigger > 0.1)
            liftPower = -gamepad2.left_trigger;
        else
            liftPower = 0;
         */
        if (gamepad2.left_stick_y >= 0.1)
            liftPower = gamepad2.left_stick_y;

        if (gamepad2.right_trigger >= 0.1) {
            s1.setPosition(.5);
            s2.setPosition(.5);
        }

        if (gamepad2.left_trigger >= 0.1) {
            s1.setPosition(1);
            s2.setPosition(1);
        }

        if (gamepad1.right_trigger > .1) {
            intakeRightPower = gamepad1.right_trigger;
        } else if (gamepad1.right_bumper) {
            intakeRightPower = -0.7;
        } else {
            intakeRightPower = 0;
        }

        if (gamepad1.left_trigger > .1) {
            intakeLeftPower = gamepad1.left_trigger;
        }
        else if (gamepad1.left_bumper)
        {
            intakeLeftPower = -0.7;
        }
        else {
            intakeLeftPower = 0;
        }

        if(gamepad1.x) {
            arm.setPosition(1);
        }

        if(gamepad1.y){
            arm.setPosition(0);
        }

        lift.setPower(-liftPower);
        intakeLeft.setPower(-intakeLeftPower);
        intakeRight.setPower(-intakeRightPower);
        frontLeft.setPower(y+x+z); //changed nov 20 from .4 to full speed
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
