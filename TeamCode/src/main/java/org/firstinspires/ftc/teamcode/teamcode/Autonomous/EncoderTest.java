package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teamcode.Libraries.AutoEncoder;
import org.firstinspires.ftc.teamcode.teamcode.Libraries.ConceptVuforiaNavigation;


/**
 * Created by Ariella on 6/10/2017.
 */
@Autonomous(name="Encoder Test", group="Pushbot")
@Disabled
public class EncoderTest extends AutoEncoder {
    @Override
    public void runOpMode() throws InterruptedException{
       super.runOpMode();
        DriveForward(.4,1680);
        StopDriving();
        sleep(5000);
    }


}
