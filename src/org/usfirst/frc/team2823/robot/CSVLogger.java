package org.usfirst.frc.team2823.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CSVLogger {
	private String m_directory;
	
	private File m_f;
	private BufferedWriter m_bw;
	private FileWriter m_fw;
	
	private boolean m_logEnabled = false;
	
	public CSVLogger(String d) {
		m_directory = d;
	}
	
	//start logging to a file, does not necessarily have to be a .csv
	public void open(String file) {
		if(m_logEnabled){
			return;
		}
		
		try {
			//assumes user has not inserted a trailing slash when setting the directory
			m_f = new File(m_directory + "/" + file);
			
			if(!m_f.exists()) {
				m_f.createNewFile();
			}
			
			m_fw = new FileWriter(m_f);
	  		
		} catch(IOException e) {
			e.printStackTrace();
		}
		
		m_bw = new BufferedWriter(m_fw);
		
		m_logEnabled = true;
	}
	
	//start logging to a file with a header
	public void open(String file, String header) {
		open(file);
		
		try {
			m_bw.write(header);
			
		} catch(IOException e) {
			e.printStackTrace();
		}
	}
	
	//try to write out the given object (most Objects should have a toString() method, and this allows for multiple print
	//	data types without overloading methods)
	public void write(Object o) {
		if(m_logEnabled) {
			try{
				m_bw.write(o.toString());
				
			} catch(IOException e) {
				System.out.println("Failed to write to file in " + m_directory);
				m_logEnabled = false;
			}
		}
	}
	
	//stop logging to file
	public void close() {
		if(m_logEnabled){
			try {
				m_bw.close();
				m_fw.close();
				
			} catch(IOException e) {
				e.printStackTrace();
			}
		}
		
		m_logEnabled = false;
	}
}
