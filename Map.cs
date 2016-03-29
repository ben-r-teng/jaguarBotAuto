using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;
        public double noWallNum = 999999;

        public Map()
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:

	        numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
	        mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

	        mapSegmentCorners[1,0,0] = -3.55/2;
	        mapSegmentCorners[1,0,1] = 0.0;
	        mapSegmentCorners[1,1,0] = -3.55/2;
	        mapSegmentCorners[1,1,1] = -2.74;

	        mapSegmentCorners[2,0,0] = 3.55/2;
	        mapSegmentCorners[2,0,1] = 0.0;
	        mapSegmentCorners[2,1,0] = 3.55/2;
	        mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
		
		        // Set wall segments to be horizontal
		        slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
		        intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)

        // WARNING: TODO: Might want to replace the slopes and intercepts for the wall maps
        public double GetWallDistance(double x, double y, double t, int segment){

	        // ****************** Additional Student Code: End   ************
            double x1 = mapSegmentCorners[segment, 0, 0];
            double y1 = mapSegmentCorners[segment, 0, 1];
            double x2 = mapSegmentCorners[segment, 1, 0];
            double y2 = mapSegmentCorners[segment, 1, 1];

            // The temporary variable representing the intersection of the wall and the ray casted
            double xf = 0;
            double yf = 0;

            //Phase 1: Check if the robot and the walls intersect
            
            // Parallel Vertical lines
            if((x1 == x2) && ((t == Math.PI/2) || (t == -Math.PI)))
            {
                return noWallNum;
            }
            // if vertical line
            else if (x1 == x2)
            {
                double mr = Math.Tan(t);
                xf = x1;
                yf = mr * (xf - x) + y;
            }
            //Vertical Robot
            else if ((t == Math.PI / 2) || (t == -Math.PI))
            {
                double mw = (y2 - y1) / (x2 - x1);
                xf = x;
                yf = mw * (x - x1) + y1;
            }
            else
            {
                double mw = (y2 - y1)/(x2 - x1);
                double mr = Math.Tan(t);
                if (mw == mr)
                {
                    return noWallNum;
                }
                xf = (mr * x - mw * x1 + y1 - y) / (mr - mw);
                yf = mr * (xf - x) + y;
            }



            //Check if ray casted in right direction
            if ((t == Math.PI / 2) || (t == -Math.PI))
            {
                if (xf > x)
                    return noWallNum;

            }
            else if (t == 0)
            {
                if (xf < x)
                    return noWallNum;
            }
            // The theta and the difference of the location and wall should match
            // eg. positive theta must correspond to a wall above the robot
            // else the wall cant collide
            else if (t * (yf - y) <= 0)
            {
                return noWallNum;
            }

            //Rounded to catch small errors
            int roundDig = 3;
            xf = Math.Round(xf, roundDig);
            yf = Math.Round(yf, roundDig);

            //Phase 2: Check if the intersection point is between the wall ends
            double largerx = Math.Round(Math.Max(x1, x2),roundDig);
            double smallerx = Math.Round(Math.Min(x1, x2),roundDig);
            double largery = Math.Round(Math.Max(y1, y2),roundDig);
            double smallery = Math.Round(Math.Min(y1, y2),roundDig);


            if (xf >= smallerx && xf <= largerx && yf >= smallery && yf <= largery)
            {
                return Math.Sqrt(Math.Pow(xf - x, 2) + Math.Pow(yf - y, 2));
            }
            else
            {
                return noWallNum;
            }
           // double yWall = mapSegment;
	        return 0;
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){


            double minDist = noWallNum;
            double currDist = noWallNum;
            for (int i = 0; i < numMapSegments; i++)
            {
                currDist = GetWallDistance(x, y, t, i);
                if (currDist < minDist)
                {
                    minDist = currDist;
                }
            }
	        // ****************** Additional Student Code: Start ************

	        // Put code here that loops through segments, calling the
	        // function GetWallDistance.

	        // ****************** Additional Student Code: End   ************

	        return minDist;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        bool CollisionFound(double n1x, double n1y, double n2x, double n2y, double tol){



	        
	        return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){
            double dist = 0;


            return dist;
        }






    }
}
