package localization;
import com.fasterxml.jackson.annotation.JsonProperty;


public class Orientation {

	@JsonProperty("W")
	public double w;
	@JsonProperty("X")
	public double x;

	@JsonProperty("Y")
	public double y;
	

	@JsonProperty("Z")
	public double z;
	
	public Orientation() {
		x = 0;
		y = 0;
		z = 0;
	}
	

}
