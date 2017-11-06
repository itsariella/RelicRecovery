
package org.firstinspires.ftc.teamcode.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp(name="Pushbot: lift test", group="Pushbot")
public class LiftTest extends OpMode {

    // Motors - temporary positions

    public DcMotor lift;

    public Servo s1;
    public Servo s2;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        lift = hardwareMap.dcMotor.get("lift");


        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        s2.setDirection(Servo.Direction.REVERSE);

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
    public void start(){

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double right;

        right = gamepad1.right_stick_y;

        if (gamepad1.right_bumper){
            s1.setPosition(.7);
            s2.setPosition(.7);
        }
        if (gamepad1.left_bumper){
            s1.setPosition(0);
            s2.setPosition(0);
        }

        if (abs(right) < .1) {
            right = 0;
        }

        lift.setPower(right);


    /*
     * Code to run ONCE after the driver hits STOP
     */
    }
    @Override
    public void stop(){
    }

}
