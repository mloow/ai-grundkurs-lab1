package localization;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * The Orientation class represents the orientation of a robot at a given time. The class is used in parsing json path
 * files to java objects.
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class Orientation {

	@JsonProperty("W")
	public double w;
	@JsonProperty("X")
	public double x;

	@JsonProperty("Y")
	public double y;
	

	@JsonProperty("Z")
	public double z;

    /**
     * Constructs a new Orientation with all angles set to zero.
     */
	public Orientation() {
		x = 0;
		y = 0;
		z = 0;
	}

}
