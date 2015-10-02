package localization;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Represents a node of a path for a robot to travel. The class is used in parsing of json path files to java objects.
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class PathNode {
	
	@JsonProperty("Pose")
	public Pose pose;
	@JsonProperty("Status")
	public String status;
	@JsonProperty("Timestamp")
	public String timestamp;

    /**
     * Constructs a new PathNode with a new Pose, status "0" and timestamp "0"
     * @see Pose
     */
	public PathNode() {
		this.pose = new Pose();
		this.status = "0";
		this.timestamp = "0";
	}

}
