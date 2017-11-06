
package org.firstinspires.ftc.teamcode.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class TankDrive extends OpMode {

    // Motors - temporary positions
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

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

        double right;
        double left;

        left = gamepad1.right_stick_y * .4;
        right = gamepad1.left_stick_y * .4;

        if(Math.abs(gamepad1.right_stick_y) < .1 )   {
            right = 0;
        }

        if(Math.abs(gamepad1.left_stick_y) <  .1) {
            left = 0;
        }

            frontRight.setPower(right);
            backRight.setPower(right);

            frontLeft.setPower(left);
            backLeft.setPower(left);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
