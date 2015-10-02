package localization;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * The class Position represents the position of a robot and i used to parse path files in json format to Java objects
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class Position {


	@JsonProperty("X")
	public double x;

	@JsonProperty("Y")
	public double y;

	@JsonProperty("Z")
	public double z;

    /**
     * Constructs a new Position at (x, y, 0).
     * @param x the x coordinate
     * @param y the y coordinate
     */
	public Position(double x, double y) {
		this.x = x;
		this.y = y;
		z = 0;
	}

    /**
     * Constructs a new Position at (0, 0, 0)
     */
	public Position() {
		x = 0; y = 0; z = 0;
	}

    /**
     * Returns the distance between the Position and the destination Position in the xy-plane.
     * @param dest the destination Position
     * @return the distance between the Position and the destination Position in the xy-plane
     */
	public double getDistanceTo(Position dest) {

		return Math.sqrt(Math.pow(dest.x-x, 2)+Math.pow(dest.y-y,2));
	}

    /**
     * Returns a string representation of the Position with the format (x, y).
     * @return a string representation of the Position
     */
	@Override
	public String toString() {

		return "(" + x + ", " +  y + ")";
	}

    /**
     * Determines if the Position is equal to a given Object. If the Object is a Position and the x's and y's of
     * both Positions are equals, the Positions are considered to be equal.
     * @param obj the Object to be determined equal or not to this Position
     * @return true if the Object is equal to this Position, false otherwise
     */
	@Override
	public boolean equals(Object obj) {
		if(obj instanceof Position) {
			return ((Position)obj).x == this.x && ((Position)obj).y == this.y;
		} else {
			return false;
		}
	}

}
