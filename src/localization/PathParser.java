package localization;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * The PathParser can parse a path in json format to an array of PathNodes
 *
 * @see PathNode
 *
 * @author Marcus Lööw (oi12mlw@cs.umu.se)
 * @author Andreas Günzel (dv13agl@cs.umu.se)
 */
public class PathParser {

	BufferedReader reader;
	ObjectMapper mapper;

	/**
	 * Constructs a new PathParser for a file with a given file path
	 * @param filePath the path to the file to be parsed
	 * @throws FileNotFoundException if the file to be parsed could not be found
	 */
	public PathParser(String filePath) throws FileNotFoundException {
		
		reader = new BufferedReader(new FileReader(filePath));
		mapper = new ObjectMapper();
	}

    /**
     * Parses the json path file to a PathNode array
     * @return the json path file parsed to a PathNode array
     * @throws IOException if the json path file could not be parsed or read
     * @see PathNode
     */
	public PathNode[] getPath() throws IOException  {

        return mapper.readValue(reader, PathNode[].class);
		
	}
}
