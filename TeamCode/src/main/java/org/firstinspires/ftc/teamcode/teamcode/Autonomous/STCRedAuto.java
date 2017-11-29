package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.AutoEncoder;


/**
 * Created by Ariella on 6/10/2017.
 */
@Autonomous(name="STC Red Auto", group="Pushbot")
@Disabled
public class STCRedAuto extends AutoEncoder {

//geared 2:1

    @Override
    public void runOpMode() throws InterruptedException{
       super.runOpMode();

        DriveBackwards(.2,1585);
        StopDriving();
        sleep(500);
        TurnRight(.2,1265);
        DriveBackwards(.2,3455);
        StopDriving();
        sleep(1000);
        StopDriving();
        sleep(500);
        DriveForward(.2,560);
        TurnLeft(.2,1210);
        DriveBackwards(.2,2445);
        TurnRight(.2,1270);
        DriveBackwards(.2,570);

    }


}
