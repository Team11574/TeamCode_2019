package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue 2 Blocks Fast No Place", group="Autonomous")
public class Blue2BlockFastNoDeliver extends LinearOpMode {




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
    final int block_forward_dist = 1800; //Distance to move foward while grabbing the block
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
        auto.wall_buffer = 2250; //will be much closer //previously 2400

        while(!isStarted()) { //needs a godo way to exit out of the loop
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings
            auto.recognize_block();



        }
        auto.start();
        auto.grabBlockFastColor(move_from_wall,block_distance,block_forward_dist);
        //auto.turnOrient(-1,0,500,0,0,0);
        auto.moveToFoundationFast(1, (line_dist_from_start), (extra_dist) );
        //auto.moveToFoundationY(-1,line_dist_from_start,extra_dist);
        {

            Robot.resetTime();



            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 800) { //need to check if some are done

                //Robot.pantagraphDown(pantTime);
                Robot.outtake(1); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
        }
        auto.turnOrient(-1,0,800); //turn to ensure direction
        int extra = 0;
        if(most_recent_position == 0) {
            extra = 700;
        }
        //------
        if (auto.most_recent_position == 2) {
            //special case, need to change heading afteer this
            //previously, was -1100
            auto.moveDirMax(-1,0,0,(int) (line_dist_from_start+(4*block_distance_x/Math.sqrt(2)) - 300),4000,0,0,0);
        }
        else {
            auto.moveDirMax(-1, 0, 0,200+ (int) (line_dist_from_start + (4 * block_distance_x / Math.sqrt(2)) - extra), 4000, 0, 0, 0);
        }
        auto.turnOrient(-1,0,800); //turn to ensure direction
        auto.moveDirMax(-1,.4,0,(int) (extra_dist*.9 - 400),4000,-.4,0,0);
        if (auto.most_recent_position == 2) {
            auto.turnOrient(-1, -.35, 800, -1, 0, 0); //turn to a special heading
            auto.moveDirMax(0, -1, 0, (int) (1250), 4000, -1, 0, 0);
            boolean grabbedBlock = false;
            double distance_traveled = 0;
            double average_motor_len = 0;
            {

                Robot.resetTime();
                double[] powers = motorPower.calcMotorsFull(0, -.8, 0);
                for (int i = 0; Robot.motors.length > i; i++) {
                    Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
                }
                Robot.setMotors(powers, 1);
                ElapsedTime pantTime = new ElapsedTime();
                while (pantTime.milliseconds()<3000 && !grabbedBlock) { //need to check if some are done
                    Robot.setMotors(powers, 1);
                    Robot.pantagraphDown(pantTime);
                    Robot.intake(1); //maybe .7 will be more relaible
                    grabbedBlock = Robot.isBlock();
                }

            }

            for (int i = 0; Robot.motors.length > i; i++) {
                average_motor_len += Math.abs(Robot.motors[i].getCurrentPosition());
                //since we travelled forward, we can just move that part backwards
            }
            average_motor_len /= 4.;
            auto.moveDirMax(0,1,0,(int) (average_motor_len+(int)( block_distance_x/Math.sqrt(2))-auto.wall_buffer),4000);
            auto.turnOrient(-1,0,450,0,0,0);//doesnt need to be that long, actually, wait until second step
            //a little extra orientation time
        }
        else {
            auto.turnOrient(-1, 0, 500, -1, 0, 0); //turn to ensure direction
            auto.moveDirMax(0, -1, 0, (int) (1250), 4000, -1, 0, 0);
            auto.intakeBlockColor(block_forward_dist,3000,(int)( block_distance_x/Math.sqrt(2)) );
        }




        auto.moveToFoundationFast(1, (line_dist_from_start)+block_distance_x*3, (extra_dist) );

        {

            Robot.resetTime();



            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 600 && !auto.timePast()) { //need to check if some are done

                //Robot.pantagraphDown(pantTime);
                Robot.outtake(1); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
        }
        auto.moveDirMax(-1,0,0,1800,5000,0,0,0); //ark
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
