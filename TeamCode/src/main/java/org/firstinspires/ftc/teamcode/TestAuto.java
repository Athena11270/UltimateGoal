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

        // try turning
        seven.turn(90, 0.5);
        seven.turn(90, 0.5);
        seven.turn(-90, 0.5);
        seven.turn(-90, 0.5);


    }
}
