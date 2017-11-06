
package org.firstinspires.ftc.teamcode.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/* This program tests autonomous
 */

@Autonomous(name="AutoMecanum", group="Pushbot")
@Disabled
public class AutoMecanum extends LinearOpMode {

    // deleted MotorController

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        //using Modern Robotics, init motorControllers // deleted


        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        //motors mirror each other
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //wait for game to start
        waitForStart();

    }


    public void DriveBackwards(double power) {

        //set power
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

    }

    public void DriveForward(double power) throws InterruptedException {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

    }

    public void TurnRight(double power){

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);

    }

    public void TurnLeft(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void StrafeLeft(double power){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void StrafeRight(double power){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }


    public void StopDriving() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
