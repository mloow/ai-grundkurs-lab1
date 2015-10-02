package given;
import java.util.Map;

public interface Response
{
   public void setData(Map<String, Object> data);
   public long getTimestamp();
   public String getPath();
}
