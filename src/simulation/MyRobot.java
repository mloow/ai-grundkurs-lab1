package simulation;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.Locale;

import geometry.Edge;
import localization.Position;
import geometry.Triangle;
import geometry.Vertex;
import given.*;

/**
 * MyRobot represents a differential drive robot that uses the follow-the-carrot path tracking algorithm to navigate
 * from start to end of a given path. MyRobot can be seen as an interface towards the Lokkaria web interface, which can
 * set linear and angular speeds and get position an orientation of a simulated robot. To communicate with the
 * web service, a RobotCommunicator is used.
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 * @see RobotCommunicator
 */
public class MyRobot {

	private double lookAheadDistance;
	private double maxSpeed;

	private Path path;
	private Path traveledPath;
    private Path carrotPoints;

	private RobotCommunicator robotCommunicator;
	private Position position;
    private int edgeIndexOfLastCarrotPoint;
    private boolean loggingEnabled;

    /**
	 * Construct a new MyRobot. The position of the robot is set to (0, 0). The look-ahead distance is set to 1.0.
	 * The max speed is set to 1.0.
	 */
	public MyRobot() {
		this.position = new Position(0, 0);
		setLookAheadDistance(1.00);
		setMaxSpeed(1.00);
	}

	/**
	 * Tells the robot to start following the path, until it has reached the end of the path.
	 * The simplified pseudo algorithm of the path following is:
	 *
	 * While the robot has not reach the end of the path
     *      If the last carrot point has been reached
	 * 		    Get the new carrot point
	 * 		Steer and drive towards the general direction of the carrot point
	 *
	 * The algorithm to find the carrot point is described in the Path class.
	 *
	 * It should be noted that the robot always is set to drive at the maximum speed at a constant time,
     * except for large steering angles, where the speed is lowered for a while. This means that the speed and
     * drive time is NOT set to make the robot travel the whole distance to the carrot point.
     * Instead, the carrot point is recalculated roughly 20 times a second,
     * and the angular speed will be set at a high rate in the direction of the carrot point.
	 *
	 * The reason for this peculiar solution is simply because it resulted faster run times,
     * than what runs where steer and drive speeds were set to make the robot "exactly" reach the carrot
     * within the drive time.
	 *
	 * @throws Exception
	 */
	public void run() throws Exception {

        if(loggingEnabled) {
            traveledPath = new Path();
            carrotPoints = new Path();
        }

		LocalizationResponse locResponse = new LocalizationResponse();
		robotCommunicator.getResponse(locResponse);
        edgeIndexOfLastCarrotPoint = 0;
        Vertex carrotPoint = getCarrotPoint();
        boolean didReachCarrotPoint = true;
		boolean done = false;

		while (!done) {
			locResponse = new LocalizationResponse();
			try {

				robotCommunicator.getResponse(locResponse);

                position = getPosition(locResponse);

                /* Only get a new carrot point if the last one was actually reached. */
                if(didReachCarrotPoint) {
                    carrotPoint = getCarrotPoint();
                }

				double errorAngle = getErrorAngle(getOrientation(locResponse), carrotPoint);

                /* The linear speed is set to max, regardless of the distance to the c.p. */
				double linearSpeed = maxSpeed;

                /* The drive time is set to a constant 50 ms, regardless of the distance to the c.p. */
				long driveTime = 50L;

				/* If the error angle (steering angle), is too great, slow the robot down for a while */
				if (Math.abs(errorAngle) > 110) {
					linearSpeed = 0.3;
					driveTime += 1000;
				}

                /* The angular speed is always set to 2.5 times the error angle, regardless of drive time */
				double angularSpeed = Math.toRadians(errorAngle)*2.5;

				setSpeed(angularSpeed, linearSpeed);

				Thread.sleep(driveTime);

				if (isCloseToFinish()) {
					done = true;
					stop();
				}

                didReachCarrotPoint = isWithinDistanceTo(0.5, carrotPoint.toPosition());

                /* Log position and carrot point */
                if(loggingEnabled) {
                    traveledPath.concatPath(new Vertex(position));
                    carrotPoints.concatPath(carrotPoint);
                }

			} catch (Exception e) {
				System.err.println("Error while running: " + e.getMessage());
                done = true;
			}
		}

	}

    /**
     * Writes logged carrot points and robot positions (traveled path) to text files.
     */
    public void writeLogs() throws FileNotFoundException, UnsupportedEncodingException {

        PrintWriter carrotPointWriter = new PrintWriter("cps.txt", "UTF-8");
        PrintWriter robotPathWriter = new PrintWriter("robotPos.txt", "UTF-8");

        for(Edge e : traveledPath.getEdges()) {
            robotPathWriter.println(String.format(Locale.US, "%.6f %.6f", e.start.x, e.start.y));
        }

        Vertex pathEnd = traveledPath.getEnd();
        if(pathEnd != null) {
            robotPathWriter.println(String.format(Locale.US, "%.6f %.6f", pathEnd.x, pathEnd.y));
        }

        for(Edge e: carrotPoints.getEdges()) {
            carrotPointWriter.println(String.format(Locale.US, "%.6f %.6f", e.start.x, e.start.y));
        }

        Vertex carrotEnd = carrotPoints.getEnd();
        if(carrotEnd != null) {
            robotPathWriter.println(String.format(Locale.US, "%.6f %.6f", carrotEnd.x, carrotEnd.y));
        }

        carrotPointWriter.close();
        robotPathWriter.close();
    }

    /**
     * Determines if the robot is close to the end of the path. If the distance from the robot to the end of the path
     * is less than 1.0 length units. Another condition that must be fulfilled for the robot to be considered close to
     * finish, is that the edge of the last carrot point is at least within the last 5% of all the edges of the path.
     *
     * @return true if the robot is close to the end of the path, false otherwise.
     */
	private boolean isCloseToFinish() {

        int lastEdges = path.getEdges().size() / 20;
		return isWithinDistanceTo(1.0, path.getEnd().toPosition())
                && isWithinEdgesFromEnd(lastEdges);
	}

    /**
     * Determines if the edge of the last carrot point is within at least a given number of edges
     * from the end of the path.
     * @param edgesFromEnd the number of edges from the end of the path
     * @return true if the edge of the last carrot point is within the given range from the end of the path,
     * false otherwise
     */
    private boolean isWithinEdgesFromEnd(int edgesFromEnd) {
        return (path.getEdges().size() - edgeIndexOfLastCarrotPoint) < edgesFromEnd;
    }


    /**
     * Sets the RobotCommunicator of the MyRobot.
     * @param robotCommunicator the RobotCommunicator
     * @see RobotCommunicator
     */
    public void setRobotCommunicator(RobotCommunicator robotCommunicator) {
		this.robotCommunicator = robotCommunicator;
	}

    /**
     * Tells the RobotCommunicator to send a request of setting the given angular and linear speed of the robot.
     * @param angularSpeed the angular speed to set
     * @param linearSpeed the linear speed to set
     * @throws Exception if there was an error sending a request
     * @see RobotCommunicator
     */
	private void setSpeed(double angularSpeed, double linearSpeed) throws Exception {
		DifferentialDriveRequest diffDriveRequest = new DifferentialDriveRequest();
		diffDriveRequest.setLinearSpeed(linearSpeed);
		diffDriveRequest.setAngularSpeed(angularSpeed);
		robotCommunicator.putRequest(diffDriveRequest);
	}

    /**
     * Calculates the error angle, or steering angle, to the given carrot point from the robot,
     * given the robots orientation.
     * @param orientation the orientation of the robot
     * @param carrotPoint the carrot point
     * @return the error angle from the robot to the carrot point in degrees
     */
	private double getErrorAngle(double orientation, Vertex carrotPoint) {

		double carrotAngle = Math.toDegrees(Math.atan2(carrotPoint.y - position.y, carrotPoint.x - position.x));
		double errorAngle = carrotAngle - orientation;

        /* Keep the angle between -180 and 180. */
		if (errorAngle > 180) {
			errorAngle -= 360;
		} else if (errorAngle < -180) {
			errorAngle += 360;
		}

		return errorAngle;
	}

    /**
     * Calculates and return the carrot point of the robot given the robots look-ahead distance and position.
     *
     * When calculating the carrot point, you can sometimes get the unwanted carrot point, even though it is the
     * correct one. For example, if your robot is just about to start following a long path, and the ending is at the
     * same spot as the start, the carrot point found may very well be the end point of the path, even though your
     * robot haven't traveled the path. To counter this problem, an interval of edges of the path is specified within
     * which the carrot point must be picked. The interval is based on the index of the last "carrot edge" that was
     * picked, and a number of edges together making up a length of 1.5 look-ahead distances.
     *
     * The algorithms to calculate the carrot point is described in the Path class.
     * @return the carrot point
     * @see Path
     */
	private Vertex getCarrotPoint() {

        int edgeIntervalLength = (int) ((lookAheadDistance / path.avgEdgeLength()) * 1.5);

        Vertex carrotPoint = path.getCarrotPointFrom(new Vertex(position), lookAheadDistance,
                edgeIndexOfLastCarrotPoint, edgeIndexOfLastCarrotPoint + edgeIntervalLength);
        edgeIndexOfLastCarrotPoint = path.getIndexOfLastCarrotEdge();

        return carrotPoint;
	}

    /**
     * Tells the RobotCommunicator to send a request to stop the forward movement and rotation of the robot.
     * @throws Exception if there was an error sending the request
     */
	private void stop() throws Exception {
		DifferentialDriveRequest dr;
		dr = new DifferentialDriveRequest();
		dr.setAngularSpeed(0);
		dr.setLinearSpeed(0);
		robotCommunicator.putRequest(dr);
	}

    /**
     * Determines if the position of the robot is within a given distance to another given position.
     * In this case The positions are treated as coordinates in a cartesian coordinate system.
     * @param distance the distance from the robot the given point should be located within
     * @param position the position
     * @return true if the position is within the given distance to the robot position
     */
	private boolean isWithinDistanceTo(double distance, Position position) {

		return this.position.getDistanceTo(position) < distance;
	}

    /**
     * Sets the maximum linear speed of the robot
     * @param maxSpeed the maximum linear speed
     */
	public void setMaxSpeed(double maxSpeed) {
		this.maxSpeed = maxSpeed;
	}

    /**
     * Sets the look-ahead distance of the robot
     * @param lookAheadDistance the look-ahead distance
     */
	public void setLookAheadDistance(double lookAheadDistance) {
		this.lookAheadDistance = lookAheadDistance;
	}

    /**
     * Gets the current position of the robot from the given LocalizationResponse
     * @param r the LocalizationResponse
     * @return the current position of the robot
     * @see LocalizationResponse
     */
	private Position getPosition(LocalizationResponse r) {

        // index 0 has x, 1 has y, 2 has z.
		double[] posArray = r.getPosition();

        return new Position(posArray[0], posArray[1]);

	}

    /**
     * Gets the current orientation of the robot from the given LocalizationResponse. The orientation is calculated
     * by making a Triangle from the origin, the bearing of the robot, and the absolute x of the bearing on the x-axis,
     * and calculating the angle in the origin vertex of the Triangle. If the bearing y is negative, the angle is
     * negated.
     * @param r the LocalizationResponse
     * @return the current orientation of the robot
     * @see Triangle
     * @see Quaternion
     */
	private double getOrientation(LocalizationResponse r) {

		double[] bearing = (new Quaternion(r.getOrientation())).bearing();

		Vertex origin = new Vertex(0, 0);
		Vertex v1 = new Vertex(bearing[0], bearing[1]);
		Vertex v2 = new Vertex(Math.abs(bearing[0]), 0);

		Triangle triangle = new Triangle(origin, v1, v2);
		try {
			double angle = triangle.getAngleInVertex(origin);
			angle = (bearing[1] < 0 ? -angle : angle);
			return angle;
		} catch (Exception e) {
			return 0;
		}

	}

    /**
     * Records the path the robot travels and writes it to a json file with the given file name.
     * The file can later be parsed by the PathParser. The intended use of this function is to call it while MRDS
     * is running on your computing, and then manually control the robot from MRDS.
     *
     * The only way the recording will stop "properly" is when the connection from the RobotCommunicator to the
     * Lokarria web service is closed, upon which an exception will be thrown and caught. When the exception is caught
     * the recording stops. A simple way to close the connection is by closing MRDS.
     *
     * @param pathName the file name to which the json representation of the path should be written
     * @see localization.PathParser
     * @see RobotCommunicator
     */
	public void record(String pathName) {
        traveledPath = new Path();
		boolean running = true;
		PrintWriter pw;

		while (running) {
			try {

				LocalizationResponse r = new LocalizationResponse();
				robotCommunicator.getResponse(r);
				Position p1 = getPosition(r);

				if(traveledPath.getLastAddedVertex() != null) {

					if(traveledPath.getLastAddedVertex().toPosition().getDistanceTo(p1) > 0.01) {
						traveledPath.concatPath(new Vertex(p1.x, p1.y));
					}

				} else {
					traveledPath.concatPath(new Vertex(p1.x, p1.y));
				}

				Thread.sleep(20);

			} catch (Exception e) {
				running = false;
			}
		}

		try {
			String json = traveledPath.toJson();
			pw = new PrintWriter(pathName);
			pw.write(json);
			pw.close();
		} catch (FileNotFoundException e) {
			System.err.println("Could not write to file " + pathName);
            return;
		} catch (IOException e) {
            System.err.println("Could not convert json");
            return;
        }

        System.err.println("DONE");
		System.err.println("Recorded " + traveledPath.getEdges().size() + 1 + " coordinates");

	}

    /**
     * Sets the path for the MyRobot to follow
     * @param path the path
     */
	public void setPath(Path path) {
		this.path = path;
	}

    /**
     * Enables logging of carrot points and robot positions.
     * @param loggingEnabled true enables logging
     */
    public void setLoggingEnabled(boolean loggingEnabled) {
        this.loggingEnabled = loggingEnabled;
    }

    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }
}
