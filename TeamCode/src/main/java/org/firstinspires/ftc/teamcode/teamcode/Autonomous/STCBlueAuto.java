package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.AutoEncoder;


/**
 * Created by Ariella on 6/10/2017.
 */
@Autonomous(name="STC Blue Auto", group="Pushbot")
@Disabled
public class STCBlueAuto extends AutoEncoder {

//geared 2:1

    @Override
    public void runOpMode() throws InterruptedException{
       super.runOpMode();

        DriveBackwards(.2,1570);
        StopDriving();
        sleep(500);
        TurnLeft(.2,1220);
        DriveBackwards(.2,3650);
        StopDriving();
        sleep(1000);
        Intake(.1,7000);
        StopDriving();
        sleep(500);
        DriveForward(.2,560);
        TurnRight(.2,1210);
        DriveBackwards(.2,2445);
        TurnLeft(.2,1250);
        DriveBackwards(.2,560);

    }


}
