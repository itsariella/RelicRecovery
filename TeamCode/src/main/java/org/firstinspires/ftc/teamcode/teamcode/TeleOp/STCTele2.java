
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="STCRobot2", group="Pushbot")
@Disabled
public class STCTele2 extends OpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    private float x;
    private float y;
    private float z;

    public DcMotor intakeLeft;
    public DcMotor intakeRight;
    public DcMotor wheelBoxLeft;
    public DcMotor wheelBoxRight;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        wheelBoxLeft = hardwareMap.dcMotor.get("wheelBoxLeft");
        wheelBoxRight = hardwareMap.dcMotor.get("wheelBoxRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelBoxRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
        double intakeIn;
        double boxValue;

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        intakeIn = gamepad1.right_trigger * 4;



        if (Math.abs(gamepad1.left_stick_y) > .1)
            y = gamepad1.left_stick_y;
        else
            y = 0;


        if (Math.abs(gamepad1.right_stick_x) > .1)
            z = gamepad1.right_stick_x;
        else
            z = 0;

        if(Math.abs(gamepad1.left_stick_x) > .1)
            x = gamepad1.left_stick_x;
        else
            x = 0;


        if (gamepad1.right_bumper)
            boxValue = 1;

        else if(gamepad1.left_bumper)
            boxValue = -1;
        else
            boxValue = 0;

        frontRight.setPower((y-z-x)*.5);
        backRight.setPower((y-z+x)*.5);
        frontLeft.setPower((y+z+x)* .5);
        backLeft.setPower((y+z-x)* .5);
        intakeRight.setPower(intakeIn);
        intakeLeft.setPower(intakeIn);
        wheelBoxLeft.setPower(boxValue);
        wheelBoxRight.setPower(boxValue);


        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
