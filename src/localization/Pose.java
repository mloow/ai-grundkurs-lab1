package localization;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * The Pose class represents a pose of a robot at a given time. The class is used to parse paths in json format to
 * java objects.
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
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
