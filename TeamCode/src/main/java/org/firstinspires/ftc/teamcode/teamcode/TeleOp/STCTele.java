
package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="STCRobot", group="Pushbot")
@Disabled
public class STCTele extends OpMode {

    // Motors

    public DcMotor Frontright;
    public DcMotor Frontleft;
    public DcMotor Backright;
    public DcMotor Backleft;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        Frontright = hardwareMap.dcMotor.get("frontright");
        Frontleft = hardwareMap.dcMotor.get("frontleft");
        Backright = hardwareMap.dcMotor.get("backright");
        Backleft = hardwareMap.dcMotor.get("backleft");


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

        double frontleft = 0;
        double frontright = 0;
        double backright = 0;
        double backleft = 0;


        frontleft = gamepad1.left_stick_y;
        backleft = gamepad1.left_stick_y;
        frontright = gamepad1.right_stick_y;
        backright = gamepad1.right_stick_y;



        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1) {


        }


        Frontleft.setPower(frontleft);
        Frontright.setPower(frontright);
        Backright.setPower(backright);
        Backleft.setPower(backleft);
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
