package localization;
import com.fasterxml.jackson.annotation.JsonProperty;


public class Position {


	@JsonProperty("X")
	public double x;

	@JsonProperty("Y")
	public double y;

	@JsonProperty("Z")
	public double z;

	public Position(double x, double y) {
		this.x = x;
		this.y = y;
		z = 0;
	}

	public Position() {
		x = 0; y = 0; z = 0;
	}

	public double getDistanceTo(Position dest) {

		return Math.sqrt(Math.pow(dest.x-x, 2)+Math.pow(dest.y-y,2));
	}

	@Override
	public String toString() {

		return "(" + x + ", " +  y + ")";
	}
	
	@Override
	public boolean equals(Object obj) {
		if(obj instanceof Position) {
			return ((Position)obj).x == this.x && ((Position)obj).y == this.y;
		} else {
			return false;
		}
	}

}
