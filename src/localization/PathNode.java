package localization;
import com.fasterxml.jackson.annotation.JsonProperty;


public class PathNode {
	
	@JsonProperty("Pose")
	public Pose pose;
	@JsonProperty("Status")
	public String status;
	@JsonProperty("Timestamp")
	public String timestamp;
	
	public PathNode() {
		this.pose = new Pose();
		this.status = "DEFAULT";
		this.timestamp = "0";
	}

}
