
package org.firstinspires.ftc.teamcode.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/* This program tests autonomous
 */

@Autonomous(name="Encoder Auto", group="Pushbot")
@Disabled
public class AutoEncoder extends LinearOpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //calculations
    double diameterOfWheel = 3.96; //inches
    double circumference = diameterOfWheel * Math.PI;
    int distanceToGo = 12;
    double rotations = distanceToGo/circumference;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");


        //motors mirror each other
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //encoders
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //wait for game to start
        waitForStart();

    }


    public void DriveBackwards(double power, int distance ) {

        int movecounts = (int)(distance * COUNTS_PER_INCH);

        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        frontLeft.setTargetPosition(movecounts);
        frontRight.setTargetPosition(movecounts);
        backRight.setTargetPosition(movecounts);
        backLeft.setTargetPosition(movecounts);

        //set mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Driving Backwards",  "Running to %7d", distance);
            telemetry.addData("Path2",  "Running at %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Path3",  "Running at %7d", frontRight.getCurrentPosition());
            telemetry.addData("Path4",  "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5",  "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }
        StopDriving();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward(double power, int distance) throws InterruptedException {

        //1240 = 1 rotation

        int movecounts = (int)(distance * COUNTS_PER_INCH);

        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        frontLeft.setTargetPosition(-movecounts);
        frontRight.setTargetPosition(-movecounts);
        backRight.setTargetPosition(-movecounts);
        backLeft.setTargetPosition(-movecounts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Driving Forward",  "Running to %7d", distance);
            telemetry.addData("Path2",  "Running at %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Path3",  "Running at %7d", frontRight.getCurrentPosition());
            telemetry.addData("Path4",  "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5",  "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }
        StopDriving();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void TurnRight(double power, int distance){


        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(-distance);
        backRight.setTargetPosition(-distance);
        backLeft.setTargetPosition(distance );

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Turning Right",  "Running to %7d", distance);
            telemetry.addData("Path2",  "Running at %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Path3",  "Running at %7d", frontRight.getCurrentPosition());
            telemetry.addData("Path4",  "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Path5",  "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        StopDriving();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnLeft(double power, int distance){


        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        frontLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

         while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Turning Left",  "Running to %7d", distance);
            telemetry.addData("Front Left",  "Running at %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right",  "Running at %7d", frontRight.getCurrentPosition());
            telemetry.addData("Back Right",  "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left",  "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void StrafeRight(double power, int distance){


        // reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        frontLeft.setTargetPosition(distance * 360);
        frontRight.setTargetPosition(-distance * 360);
        backRight.setTargetPosition(distance * 360);
        backLeft.setTargetPosition(-distance * 360);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // at 12 Volts battery, .4 speed, 1240 sleep time, turns approx. 90 deg

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            telemetry.addData("Strafing Right",  "Running to %7d", distance);
            telemetry.addData("Front Left",  "Running at %7d", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right",  "Running at %7d", frontRight.getCurrentPosition());
            telemetry.addData("Back Right",  "Running at %7d", backRight.getCurrentPosition());
            telemetry.addData("Back Left",  "Running at %7d", backLeft.getCurrentPosition());
            telemetry.update();

            idle();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void StopDriving() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

}
