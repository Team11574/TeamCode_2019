package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Just Park Right", group="Autonomous")
public class JustPark extends LinearOpMode {




    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;


    //you have grabbed the black, hopefully will allow the robot to avoid the other robot
    final int line_dist_from_start = (int) (1000*1.3); //distance from start to the line under the skybridge
    final int block_distance = 1900; //Distance from the middle block to either the rigth or left block
    final int block_distance_x = 780;
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 1600; //Distance to move foward while grabbing the block
    final int extra_dist = (int) (2850*1.6)-1800;





    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }
    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);
        Robot r =new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        auto.wall_buffer = 1950; //will be much closer

        while(!isStarted()) { //needs a godo way to exit out of the loop
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings




        }

        auto.moveDirMax(-1,0,0,line_dist_from_start,3500,0,0,0);
        //might be able to place
        //

        //not gunna work, butmight as well test it
        // auto.moveBack2(0,-1,(int) (line_dist_from_start + extra_dist+block_distance_x*4) );
        //auto.turnOrient(-1,0,1000); //orient back towards blocks


        //now coudl continue







    }

    public class StopException extends RuntimeException {
        public StopException() {super();}
    }



}
