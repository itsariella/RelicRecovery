package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.ConceptVuforiaNavigation;


/**
 * Created by Ariella on 6/10/2017.
 */
@Autonomous(name="Encoder Test", group="Pushbot")
public class EncoderTest extends ConceptVuforiaNavigation {
    @Override
    public void runOpMode() throws InterruptedException{
       super.runOpMode();
        DriveForward(.4,1120);
        StopDriving();
        sleep(5000);
    }


}
