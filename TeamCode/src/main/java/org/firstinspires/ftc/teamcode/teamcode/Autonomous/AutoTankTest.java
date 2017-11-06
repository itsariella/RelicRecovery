package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.AutoTank;


/**
 * Created by Ariella on 6/10/2017.
 */
@Autonomous(name="Auto Tank Test", group="Pushbot")
@Disabled
public class AutoTankTest extends AutoTank {
    @Override
    public void runOpMode() throws InterruptedException{
       super.runOpMode();
        DriveForward(.4);
        sleep(1500);
        StopDriving();
        sleep(3000);
        TurnRight(.4);
        sleep(1240);
        StopDriving();
        sleep(22000);

    }


}
