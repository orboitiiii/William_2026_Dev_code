package frc.robot.framework;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

public class CSVLogWriter {
  private PrintWriter mWriter;
  private final String mFileName;
  private final List<String> mLogFields = new ArrayList<>();

  public CSVLogWriter(String fileName) {
    mFileName = fileName;
  }

  public void addField(String fieldName) {
    mLogFields.add(fieldName);
  }

  public void init() {
    try {
      mWriter = new PrintWriter(new FileWriter(mFileName + ".csv", true));
      mWriter.println(String.join(",", mLogFields));
      mWriter.flush();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void log(List<Object> values) {
    if (mWriter != null) {
      List<String> stringValues = new ArrayList<>();
      for (Object o : values) {
        stringValues.add(o.toString());
      }
      mWriter.println(String.join(",", stringValues));
      mWriter.flush();
    }
  }

  public void close() {
    if (mWriter != null) {
      mWriter.close();
    }
  }
}
