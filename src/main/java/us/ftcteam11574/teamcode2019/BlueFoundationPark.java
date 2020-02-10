package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Foundation Park (Testing)", group="Autonomous")
public class BlueFoundationPark extends LinearOpMode {

    //I think this would work for red too


    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int foundation_dist = 2000;
    final int foundation_buff = 500;
    final int foundation_pull = 1100;
    final int side_dist = 900;
    final int side_dir = 1;


    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof Robot.StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }
    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap, gamepad1, gamepad2);
        //Robot.pistonHome(); //coudl include this?

        Robot r = new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        Robot.pistonHome();
        while (!isStarted()) {

            //wait

        }
        Robot.resetTime();
        Robot.reset_pow();

        //move down for a bit longer
        //auto.moveDirMax(0,1,0,foundation_dist,3000,0,0,-1); //move servo down and move towards foundation
        auto.moveDirMax(-.2,1,0,foundation_dist,3000,0,0,-1); //move servo down and move towards foundation
        //auto.moveDirMax(1,0,0,400,500,0,0,-.6);

        auto.moveDir(0,.5,0,foundation_buff,3000,0,0,-1); //move
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 500  ) { //one second less, since running it earlier
                Robot.runServoDown();
            }
            Robot.reset_pow();
        }
        auto.moveDirMax(0,-1,0,foundation_dist,3000);
        auto.turnOrient(0,1,4000,0,0,0); //
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 3500  ) { //one second less, since running it earlier
                Robot.runServoUp();
            }
            Robot.reset_pow();
        }


        auto.moveDirMax(-.75,-1,0,600,4500,0,0,0); //need to test this to see if it works






    }



}
