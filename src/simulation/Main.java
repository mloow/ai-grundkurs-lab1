package simulation;
import geometry.Edge;
import given.RobotCommunicator;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.Locale;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;

import localization.PathNode;
import localization.PathParser;

/**
 * Main constructs a new MyRobot and gives it a RobotCommunicator.
 * The program arguments are then parsed. The user can choose to run the path tracking algorithm
 * for a given path, look-ahead distance and speed. To do this, the user shall run the program with the arguments:
 * -run <path file> <look-ahead distance> <speed> [-log]
 * The optional -log argument will set the program to log the path, taken path, and carrot points of the MyRobot.
 *
 * The user can also choose to record their own path. To do this, the user shall run the program with the arguments:
 * -record <outfile>
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 * @see RobotCommunicator
 * @see MyRobot
 * @see Path
 * @see PathParser
 */
public class Main {

	private static final String ARG_STRING = " { -record <outfile> | " +
			"-run <pathfile> <look-ahead distance> <speed> [-log] } ";

	public static void main(String[] args) {

		MyRobot robot = new MyRobot();
		robot.setRobotCommunicator(new RobotCommunicator("http://127.0.0.1", 50000));

		if(args[0].equals("-run")) {
			if(args.length >= 4) {

				String pathFile = args[1];
				double lookAheadDistance = Double.parseDouble(args[2]);
				double maxSpeed = Double.parseDouble(args[3]);

				if(args.length == 5 && args[4].equals("-log")) {
					robot.setLoggingEnabled(true);
                    System.err.println("Logging enabled");
                } else {
					robot.setLoggingEnabled(false);
				}

				run(robot, pathFile, lookAheadDistance, maxSpeed);

			} else {
				System.err.println("Invalid argument count");
				System.err.println("Use arguments: " + ARG_STRING);
			}

		} else if (args[0].equals("-record")) {

			if(args.length == 2) {
				recordPath(robot, args[1]);

			} else {
				System.err.println("Invalid argument count");
				System.err.println("Use arguments: " + ARG_STRING);
			}

		} else {
			System.err.println("Invalid arguments");
			System.err.println("Use arguments: " + ARG_STRING);
		}

		System.err.println("Exiting...");

	}

	/**
	 * Parses a path file, sets the resulting path to a MyRobot, and tells the MyRobot to start following the path
     * using a given look-ahead distance and maximum speed.
	 * The elapsed time it takes the MyRobot to finish is printed.
	 *
	 * @param robot the MyRobot to follow the path
	 * @param pathFile the path to the path file
	 * @param lookAheadDistance the look-ahead distance of the MyRobot
	 * @param maxSpeed the max speed of the MyRobot
	 */
	private static void run(MyRobot robot, String pathFile, double lookAheadDistance, double maxSpeed) {

		System.err.print("Reading path " + pathFile +"... ");
		Path path;
		PathParser parser;
		try {
			parser = new PathParser(pathFile);
			PathNode[] pathNodes = parser.getPath();
			path =  Path.fromPathNodes(pathNodes);
			robot.setPath(path);
		} catch (FileNotFoundException e) {
			System.err.println("\nCould not find path file");
			return;
		} catch (JsonParseException | JsonMappingException e) {
			System.err.println("\nCould not parse path file");
			return;
		} catch (IOException e) {
			System.err.println("\nError reading path file");
			return;
		}

		System.err.println("Done!");

		System.err.println(String.format("Setting look-ahead distance to %.2f", lookAheadDistance));
		System.err.println(String.format("Setting max speed to %.2f", maxSpeed));

		robot.setLookAheadDistance(lookAheadDistance);
		robot.setMaxSpeed(maxSpeed);

		countdown();

		System.err.println("RUNNING");
		double currentTime = System.nanoTime();
		try {
			robot.run();
		} catch (Exception e) {
			System.out.println("Unknown error");
		}
		double elapsed = System.nanoTime() - currentTime;

		System.err.println("DONE");
		System.err.println("Elapsed: " + elapsed / 1000000000.0 + " s");

		if(robot.isLoggingEnabled()) {
			try {
                System.err.print("Writing logs...");
                robot.writeLogs();
				writePath(path);
                System.err.println(" Done!");
            } catch (FileNotFoundException e) {
                System.err.println("Error writing logs.");
            } catch (UnsupportedEncodingException e) {
                e.printStackTrace();
            }
        }
	}

	/**
	 * Simple 3 second count down.
	 */
	private static void countdown() {

		System.err.println("Starting in...");
		try {
			System.err.println("3...");
			Thread.sleep(1000);
			System.err.println("2...");
			Thread.sleep(1000);
			System.err.println("1...");
			Thread.sleep(1000);

		} catch (Exception e) {

		}
	}


	/**
	 * Starts to record a MyRobots path to a file with the name filename.
	 * @param robot a MyRobot which path to record
	 * @param filename a file name to which the path will be written
	 */
	private static void recordPath(MyRobot robot, String filename) {
		System.err.println("Recording new path to " + filename);
		countdown();
		System.err.println("RECORDING");
		robot.record(filename);
	}

	/**
	 * Writes x and y of each Vertex of a Path to a text file for easy plotting in MATLAB
	 * @param path the Path
	 * @throws FileNotFoundException
	 * @throws UnsupportedEncodingException
	 */
	private static void writePath(Path path) throws FileNotFoundException, UnsupportedEncodingException {

		PrintWriter pw = new PrintWriter("path.txt");

		for(Edge e : path.getEdges()) {
			pw.println(String.format(Locale.US, "%.6f %.6f", e.start.x, e.start.y));
		}

		pw.close();

	}

}
