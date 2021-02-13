package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Competition_Auto", group="Linear Opmode")
public class Competition_Auto extends LinearOpMode{
    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        seven.InitCamera();
        waitForStart();


        //move to be able to see the stack size
        seven.drive(-4, 0.25);
        seven.strafeL(11, 0.5); //strafe right to align cam w/ rings
        seven.drive(-32, 0.5); //drive back to align

        //read the stack
        String stackSize = seven.DetectStackSize();
        seven.CloseCamera();

        seven.drive(-24, 0.5);
        seven.strafeR(4.5, 0.5);

        int wait = 1000;

        seven.launcherMono(1);
        sleep(wait);
        seven.bumper(true);
        sleep(wait);
        seven.bumper(false);
        seven.strafeL(7.5, 0.5);
        sleep(wait);
        seven.bumper(true);
        sleep(wait);
        seven.bumper(false);
        seven.strafeL(7, 0.5);
        sleep(wait);
        seven.bumper(true);
        sleep(wait);
        seven.bumper(false);
        seven.launcherMono(0);

            //reminder: we're driving backwards, probably have to turn to drop in C
//        if (stackSize.equals(seven.RINGS_FOUR)){
//            //move to zone C, drop wobble (15 points), move to shooting position
//            seven.drive(-24, 0.5);
//            //turn 180 :)
//            seven.drive(72, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.drive(-40, 0.5);
//            //turn 180 again :)
//            seven.strafeL(12, 0.5); //get to shot zone
//
//            //now to blast it
//            seven.launcherMono(1);
//            //bonk ring1 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//            //reposition
//            //bonk ring 2 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//            //reposition
//            //bonk ring 3 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//
//            //scoot to launch line
//
//        }
//        else if (stackSize.equals(seven.RINGS_ONE)){
//            //move to zone B, drop wobble (15 points), move to shooting position
//            seven.drive(-56, 0.5);
//            seven.strafeL(20, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.drive(32, 0.5);
//            seven.strafeL(12, 0.5);
//        }
//        else if (stackSize.equals(seven.RINGS_NONE)){
//            //move to zone A, drop wobble (15 points), move to shooting position
//            seven.drive(-24, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.strafeL(32, 0.5); //move laterally to power shot zone
//        }
//        else {
//            //uh idk I mean there's an error if we got here,
//            //so we should just do the easiest goal and pray for the best I guess
//        }

        //either we do the high goal and run the same thing thrice (12/24/36 points)
        //or we shoot & adjust thrice to hit the power shots (15/30/45 points)
        //move to park on line (5 points)
        //fully optimized, that gives us 65 points (assuming power shots).
    }
}
