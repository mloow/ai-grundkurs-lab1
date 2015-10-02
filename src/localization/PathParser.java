package localization;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;


public class PathParser {

	BufferedReader reader;
	ObjectMapper mapper;
	
	public PathParser(String filePath) throws FileNotFoundException {
		
		reader = new BufferedReader(new FileReader(filePath));
		mapper = new ObjectMapper();
	}

	public PathNode[] getPath() throws JsonParseException, JsonMappingException, IOException  {		
		
		PathNode[] path = mapper.readValue(reader, PathNode[].class);
		
		return path;
		
	}
	
}
