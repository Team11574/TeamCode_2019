package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Blue 2 Blocks Fast", group="Autonomous")
public class Blue2BockFast extends LinearOpMode {




    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;


    //you have grabbed the black, hopefully will allow the robot to avoid the other robot
    final int line_dist_from_start = (int) (2000*1.3); //distance from start to the line under the skybridge
    final int block_distance = 1900; //Distance from the middle block to either the rigth or left block
    final int block_distance_x = 780;
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 1300; //Distance to move foward while grabbing the block
    final int extra_dist = (int) (2850*1.6) + 500;





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
        auto.wall_buffer = 2250; //will be much closer

        while(!isStarted()) { //needs a godo way to exit out of the loop
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings
            auto.recognize_block();



        }

        auto.grabBlockFastColor(move_from_wall,block_distance,block_forward_dist);
        //auto.turnOrient(-1,0,500,0,0,0);
        auto.moveToFoundationFast(1, (line_dist_from_start), (extra_dist) );
        //auto.moveToFoundationY(-1,line_dist_from_start,extra_dist);
        auto.moveDirMax(0,-1,0,500,500,0,0,0);
        {

            Robot.resetTime();


            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 600) { //need to check if some are done

                //Robot.pantagraphDown(pantTime);
                Robot.outtake(1); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
            Robot.resetTime();
        }
        auto.moveDirMax(0,1,0,600,1000,0,0,0);
        auto.turnOrient(-1,0,800); //turn to ensure direction
        auto.moveDirMax(-1,0,0,(int) (line_dist_from_start+block_distance_x*2.5),4000,0,0,0);
        auto.turnOrient(-1,0,800); //turn to ensure direction
        auto.moveDirMax(-1,.3,0,(int) (extra_dist*.9),4000,0,0,0);
        auto.moveDirMax(0,-1,0,(int) (850),4000,-1,0,0);
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.8, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds()<3000) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake(1); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
        }
        auto.moveDirMax(0,1,0,block_forward_dist,2000,1,0,0);
        auto.moveToFoundationFast(1, (line_dist_from_start)+block_distance_x*3, (extra_dist) );
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
