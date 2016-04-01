import java.awt.*;

/**
 * This Class provides many useful algorithms for Robot Path Planning. It uses optimization techniques and knowledge
 * of Robot Motion in order to calculate smooth path trajectories, if given only discrete waypoints. The Benefit of these optimization
 * algorithms are very efficient path planning that can be used to Navigate in Real-time.
 *
 * This Class uses a method of Gradient Decent, and other optimization techniques to produce smooth Velocity profiles
 * for every wheel of a 4 wheeled swerve drive.
 *
 * This Class does not attempt to calculate quintic or cubic splines for best fitting a curve. It is for this reason, the algorithm can be ran
 * on embedded devices with very quick computation times.
 *
 * The output of this function are independent velocity profiles for the each wheel of a 4 wheeled swerve drive. The velocity
 * profiles start and end with 0 velocity and acceleration and maintain smooth transitions throughout the path.
 */
public class PathPlanner
{
    //The waypoint only path
    private static double[][] origPath;

    //The origPath with injected points to allow for smooth transitions
    private double[][] smoothPath;

    //Swerve paths generated based off smoothPath
    private double[][] leftUpperPath;
    private double[][] rightUpperPath;
    private double[][] leftLowerPath;
    private double[][] rightLowerPath;

    private static double[] acceleration;
    private static double[] velocity;
    private static double[] position;

    double time;

    //Parameters for tuning the path generated
    double pathAlpha;
    double pathBeta;
    double pathTolerance;

    /**
     * Constructor, takes a Path of Way Points defined as a double array of column vectors representing the global
     * cartesian points of the path in {x,y,heading} coordinates. The waypoint are traveled from one point to the next in sequence.
     *
     * For example: here is a properly formated waypoint array
     *
     * double[][] waypointPath = new double[][]{
     {1, 1, 0},
     {5, 1, 45},
     {9, 12, 90},
     {12, 9, 135},
     {15,6, 180},
     {15, 4, 225}
     };
     This path goes from {1,1,0} -> {5,1,45} -> {9,12,90} -> {12,9, 135} -> {15,6,180} -> {15,4,225}
     The units of these coordinates are position units assumed by the user (i.e inch, foot, meters)
     */
    public PathPlanner(double[][] path)
    {
        origPath = doubleArrayCopy(path);

        //default values DO NOT MODIFY;
        pathAlpha = 0.7;
        pathBeta = 0.3;
        pathTolerance = 0.0000001;
    }

    /**
     * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
     *
     * BigO: Order N x M
     */
    public static double[][] doubleArrayCopy(double[][] arr)
    {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for(int i = 0; i < arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            System.arraycopy(arr[i], 0, temp[i], 0, arr[i].length);
        }

        return temp;

    }

    /**
     * Method upsamples the Path by linear injection. The result providing more waypoints along the path.
     *
     * BigO: Order N * injection#
     */
    public double[][] inject(double[][] orig, int numToInject)
    {
        //create extended 2 Dimensional array to hold additional points
        double[][] morePoints = new double[orig.length + ((numToInject)*(orig.length-1))][8];

        int index = 0;

        //loop through original array
        for(int i = 0; i < orig.length-1; i++)
        {
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            morePoints[index][2] = orig[i][2];
            index++;

            for(int j = 1; j < numToInject + 1; j++)
            {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][0] = j*((orig[i+1][0]-orig[i][0])/(numToInject+1))+orig[i][0];

                //calculate intermediate y points  between j and j+1 original points
                morePoints[index][1] = j*((orig[i+1][1]-orig[i][1])/(numToInject+1))+orig[i][1];

                //calculate intermediate heading points  between j and j+1 original points
                morePoints[index][2] = j*((orig[i+1][2]-orig[i][2])/(numToInject+1))+orig[i][2];

                index++;
            }
        }

        //copy last
        morePoints[index][0] = orig[orig.length-1][0];
        morePoints[index][1] = orig[orig.length-1][1];
        morePoints[index][2] = orig[orig.length-1][2];

        return morePoints;
    }

    /**
     * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
     * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
     * converge. If this happens, try increasing the tolerance level.
     *
     * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
     */
    public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance)
    {
        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i = 1; i < path.length-1; i++) {
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        return newPath;
    }

    /**
     * This method calculates the optimal parameters for determining what amount of nodes to inject into the path
     * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
     * results for the final smooth path.
     *
     * Big O: Constant Time
     */
    public int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep)
    {
        int first = 0;
        int second = 0;
        int third = 0;

        double oldPointsTotal = 0;

        int[] ret;

        double totalPoints = maxTimeToComplete/timeStep;

        if (totalPoints < 100)
        {
            double pointsFirst;
            double pointsTotal;


            for (int i=4; i<=6; i++)
                for (int j=1; j<=8; j++)
                {
                    pointsFirst = i *(numNodeOnlyPoints-1) + numNodeOnlyPoints;
                    pointsTotal = (j*(pointsFirst-1)+pointsFirst);

                    if(pointsTotal<=totalPoints && pointsTotal>oldPointsTotal)
                    {
                        first=i;
                        second=j;
                        oldPointsTotal=pointsTotal;
                    }
                }

            ret = new int[] {first, second, third};
        }
        else
        {

            double pointsFirst;
            double pointsSecond;
            double pointsTotal;

            for (int i=1; i<=5; i++)
                for (int j=1; j<=8; j++)
                    for (int k=1; k<8; k++)
                    {
                        pointsFirst = i *(numNodeOnlyPoints-1) + numNodeOnlyPoints;
                        pointsSecond = (j*(pointsFirst-1)+pointsFirst);
                        pointsTotal =  (k*(pointsSecond-1)+pointsSecond);

                        if(pointsTotal<=totalPoints)
                        {
                            first=i;
                            second=j;
                            third=k;
                        }
                    }

            ret = new int[] {first, second, third};
        }

        return ret;
    }


    /**
     * Calculates the wheel paths based on robot track width and length
     *
     * Big O: 2N
     */
    private void Paths(double[][] smoothPath, double robotTrackWidth, double robotTrackLength)
    {
        this.leftLowerPath = new double[smoothPath.length][2];
        this.rightLowerPath = new double[smoothPath.length][2];
        this.leftUpperPath = new double[smoothPath.length][2];
        this.rightUpperPath = new double[smoothPath.length][2];

        for(int i = 0; i < smoothPath.length; i++) {

            smoothPath[i][2] = Math.toRadians(smoothPath[i][2]);

            this.leftLowerPath[i][0] = -(robotTrackWidth / 2 * Math.cos(smoothPath[i][2] - Math.PI / 2)) - (robotTrackLength / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][0];
            this.leftLowerPath[i][1] = -(robotTrackWidth / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) - (robotTrackLength / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][1];

            this.rightUpperPath[i][0] = (robotTrackWidth / 2 * Math.cos(smoothPath[i][2] - Math.PI / 2)) + (robotTrackLength / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][0];
            this.rightUpperPath[i][1] = (robotTrackWidth / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) + (robotTrackLength / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][1];

            this.leftUpperPath[i][0] = -(robotTrackWidth / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) - (robotTrackLength / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][0];
            this.leftUpperPath[i][1] = -(robotTrackWidth / 2 * Math.sin(smoothPath[i][2] - Math.PI / 2)) - (robotTrackLength / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][1];

            this.rightLowerPath[i][0] = (robotTrackWidth / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) + (robotTrackLength / 2 * Math.sin(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][0];
            this.rightLowerPath[i][1] = (robotTrackWidth / 2 * Math.sin(smoothPath[i][2] - Math.PI / 2)) + (robotTrackLength / 2 * Math.cos(smoothPath[i][2] + Math.PI / 2)) + smoothPath[i][1];

            smoothPath[i][2] = Math.toDegrees(smoothPath[i][2]);
        }

    }
    /**
     * Computes the motion profile for the path given a
     *
     * @param path - the smooth path for this unit to follow
     * @param maxA - max acceleration
     * @param dt - the frequency at which the robot controller is running on the robot.
     */

    public void computeMotionProfile(double[][] path, double maxA, double dt)
    {
        time = Math.sqrt(2*Math.PI*distance(path)/maxA);
        double k1 = 2*Math.PI/time;
        double k2 = maxA / k1;
        double k3 = 1/k1;

        double steps = time / dt;

        position = new double[(int) Math.ceil(steps)];
        velocity = new double[(int) Math.ceil(steps)];
        acceleration = new double[(int) Math.ceil(steps)];

        for(int i = 0; i < steps; i++)
        {
            acceleration[i] = maxA*Math.sin(k1*i*dt);
            velocity[i] = k2*(1-Math.cos(k1*i*dt));
            position[i] = k2*(i*dt - k3*Math.sin(k1*i*dt));
        }
    }

    public double distance(double[][] path)
    {
        double distance = 0;
        for(int i = 0; i < path.length - 1; i++)
        {
            if (i != 0) {
                double dx = path[i + 1][0] - path[i][0];
                double dy = path[i + 1][1] - path[i][1];

                distance += Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            }
        }
        return distance;
    }

    /**
     * This code will calculate a smooth path based on the program parameters. If the user doesn't set any parameters, the will use the defaults optimized for most cases. The results will be saved into the corresponding
     * class members. The user can then access .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity, .smoothRightVelocity, .smoothLeftVelocity as needed.
     *
     * After calling this method, the user only needs to pass .smoothRightVelocity[1], .smoothLeftVelocity[1] to the corresponding speed controllers on the Robot, and step through each setPoint.
     *
     * @param totalTime - time the user wishes to complete the path in seconds. (this is the maximum amount of time the robot is allowed to take to traverse the path.)
     * @param timeStep - the frequency at which the robot controller is running on the robot.
     * @param robotTrackWidth - distance between left and right side wheels of a skid steer chassis. Known as the track width.
     */
    public void calculate(double totalTime, double timeStep, double robotTrackWidth, double robotTrackLength, double maxA)
    {
        /**
         * pseudo code
         *
         * 1. Reduce input waypoints to only essential (direction changing) node points
         * 2. Calculate how many total datapoints we need to satisfy the controller for "playback"
         * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number
         *    of datapoints, and which follows the waypoint path.
         * 4. Calculate the motion profile
         */

        //Figure out how many nodes to inject
        int[] inject = injectionCounter2Steps(origPath.length, totalTime, timeStep);

        //iteratively inject and smooth the path
        for(int i = 0; i < inject.length; i++)
        {
            if(i == 0)
            {
                smoothPath = inject(origPath,inject[0]);
                smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
            }
            else
            {
                smoothPath = inject(smoothPath,inject[i]);
                smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
            }
        }

        //calculate UL, UP, LL, LR paths from smoothPath
        Paths(smoothPath, robotTrackWidth, robotTrackLength);

        computeMotionProfile(smoothPath, maxA, timeStep);
    }

    //main program
    public static void main(String[] args)
    {
        long start = System.currentTimeMillis();

        //create waypoint path, x y  heading
        double[][] waypoints = new double[][]{
                {2, 2, 0},
                {2, 12, 0},
                {5, 17, 45}
        };

        double totalTime = 5; //seconds
        double timeStep = .01; //period of control loop on Rio, seconds
        double robotTrackWidth = 2; //distance between left and right wheels, feet
        double robotTrackLength = 2;
        double maxAcceleration = 6;

        final PathPlanner path = new PathPlanner(waypoints);

        path.calculate(totalTime, timeStep, robotTrackWidth, robotTrackLength, maxAcceleration);

        System.out.println("Time in ms: " + (System.currentTimeMillis()-start));

        if(!GraphicsEnvironment.isHeadless())
        {
            LinePlot fig2 = new LinePlot(acceleration, Color.blue, Color.blue);
            fig2.addData(velocity, Color.magenta, Color.magenta);
            fig2.addData(position, Color.red, Color.red);

            fig2.setXLabel("time (deci-seconds)");
            fig2.setTitle("Motion Profile");
            fig2.yGridOn();
            fig2.xGridOn();
            LinePlot fig1 = new LinePlot(origPath, Color.blue, Color.green);

            //run a "simulation" of the paths with a "robot"
            for(int i = 0; i < path.leftUpperPath.length; i++)
            {
                fig1.g.setLocation(0,0); //make a static position for the window
                fig1.g.setSize(750, 750);
                fig1.yGridOn();
                fig1.xGridOn();
                fig1.setYLabel("Y (feet)");
                fig1.setXLabel("X (feet)");
                fig1.setTitle("Shows global position of robot path, along with left and right wheel trajectories");

                //force graph to show 1/2 field dimensions of 24ft x 27 feet
                fig1.setXTic(0, 27, 1);
                fig1.setYTic(0, 24, 1);

                //draw our paths for each wheel
                fig1.addData(path.smoothPath, Color.orange);
                fig1.addData(path.leftUpperPath, Color.magenta);
                fig1.addData(path.leftLowerPath, Color.magenta);
                fig1.addData(path.rightUpperPath, Color.magenta);
                fig1.addData(path.rightLowerPath, Color.magenta);

                //draw our "robot"
                double[][] topline = new double[][] {{path.leftUpperPath[i][0], path.leftUpperPath[i][1]}, {path.rightUpperPath[i][0], path.rightUpperPath[i][1]}};
                fig1.addData(topline, Color.black);

                double[][] leftline = new double[][] {{path.leftUpperPath[i][0], path.leftUpperPath[i][1]}, {path.leftLowerPath[i][0], path.leftLowerPath[i][1]}};
                fig1.addData(leftline, Color.black);

                double[][] rightline = new double[][] {{path.rightUpperPath[i][0], path.rightUpperPath[i][1]}, {path.rightLowerPath[i][0], path.rightLowerPath[i][1]}};
                fig1.addData(rightline, Color.black);

                double[][] bottomline = new double[][] {{path.leftLowerPath[i][0], path.leftLowerPath[i][1]}, {path.rightLowerPath[i][0], path.rightLowerPath[i][1]}};
                fig1.addData(bottomline, Color.black);

                //Throttle the speed of the simulation to run at abput the speed the robot would follow the path
                try {
                    //Robot simulated driving with the profiled motion
                    //NOTE: real life robot experiences *will* change.
                    Thread.sleep((long) (1000*(path.time / path.smoothPath.length)));
                } catch(Exception e) {
                    e.printStackTrace();
                }

                //Clear displayed data to update it
                if(i < path.leftLowerPath.length - 1)
                    fig1.clearData();
            }
        }
    }
}