package localization;
import com.fasterxml.jackson.annotation.JsonProperty;


public class Pose {
	
	@JsonProperty("Orientation")
	public Orientation orientation;
	@JsonProperty("Position")
	public Position position;
	
	public Pose() {
		this.orientation = new Orientation();
		this.position = new Position();
	}
	
}
