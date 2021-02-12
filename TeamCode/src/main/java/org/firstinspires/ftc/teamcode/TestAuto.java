package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Auto", group="Linear Opmode")
public class TestAuto extends LinearOpMode{
    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        waitForStart();

        // drive in a 2 foot square
        seven.drive(24, 0.5);
        seven.strafeL(24, 0.5);
        seven.drive(-24, 0.5);
        seven.strafeR(24, 0.5);


    }
}
