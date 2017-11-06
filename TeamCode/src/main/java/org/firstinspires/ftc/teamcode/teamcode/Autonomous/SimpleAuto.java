
package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.PushbotHardware;

/* This program tests autonomous
 */

@Autonomous(name="AutoMecanum", group="Pushbot")
@Disabled
public class SimpleAuto extends LinearOpMode {

    // Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public enum FieldColor{BLUE,RED}

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;

    //Instance Variables
    private BNO055IMU gryo;
    private FieldColor color;
    private PushbotHardware hardware;
    private int colorConst;
    private int ultraCheck;

    protected void initVariation (FieldColor color)
    {
        this.color = color;

        switch (color)
        {
            case BLUE:
                colorConst = 1;
                break;
            case RED:
                colorConst = -1;
                break;
        }
    }

    private int convertToCount(double inches)
    {
        return (int) (inches * COUNTS_PER_INCH);
    }

    private void checkColor() {
        switch (color) {
            case RED:
                // servo do something
            case BLUE:
                //servo do something
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        //using Modern Robotics, init motorControllers // deleted


        //wait for game to start
        waitForStart();

    }
}
