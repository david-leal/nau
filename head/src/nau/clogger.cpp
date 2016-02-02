#include "nau/clogger.h"

#include <sys/timeb.h>
#include <ctime>

const std::vector<std::string> CLogger::LogNames = { "CONFIG", "INFO", "WARN", "ERROR", "CRITICAL", "TRACE" };
std::map <CLogger::LogLevel, CLogHandler> CLogger::Logs;

CLogger::CLogger(void) {

}


CLogger::~CLogger(void) {

}


void 
CLogger::AddLog (LogLevel level, std::string file) {

	if (Logs.count(level) == 0) 
		Logs[level] = CLogHandler();
	
	Logs[level].setFile(file);
}


bool
CLogger::HasLog(LogLevel ll) {

	return 0 != Logs.count(ll);
}


void 
CLogger::Log (LogLevel logLevel, std::string sourceFile, int line, std::string message) {
	
	// log exists? If not create 
	if (Logs.count(logLevel) == 0)
		Logs[logLevel] = CLogHandler();

	timeb time;
	std::string result;
	char tempBuffer[256];

	result += "[";
	result += LogNames[logLevel];
	result += "]";

	ftime(&time);
	struct tm*	tempTm = localtime(&time.time);
	strftime(tempBuffer, 255, "(%d/%m/%Y %H:%M:%S.", tempTm);
	
	result += tempBuffer;
	sprintf (tempBuffer,"%d", time.millitm);
	result += tempBuffer;
	result += ")";

	result += "[";
	result += sourceFile;
	result += "]";

	result += "(";
	sprintf (tempBuffer, "%d", line);
	result += tempBuffer;
	result += ")";

	result += ":";

	result += message;

	result += "\n";

	Logs[logLevel].log(result);
}


void 
CLogger::LogSimple (LogLevel logLevel, std::string message) {
	
	// log exists?
	if (Logs.count(logLevel) == 0)
		Logs[logLevel] = CLogHandler();

	std::string result = message;

	result += "\n";

	Logs[logLevel].log(result);
}


void 
CLogger::LogSimpleNR (LogLevel logLevel, std::string message) {
	
	// log exists?
	if (Logs.count(logLevel) == 0)
		Logs[logLevel] = CLogHandler();

	Logs[logLevel].log(message);
}


void 
CLogger::CloseLog(LogLevel ll) {

	if (Logs.count(ll) == 0)
		return;

	Logs[ll].close();
}


// ----------------------------------------------------------
//			CLogHandler
// ----------------------------------------------------------


CLogHandler::CLogHandler() {

	m_FileName = "";
	m_FileHandler = stdout;
}


CLogHandler::~CLogHandler() {

	if (m_FileHandler != stdout)
		fclose(m_FileHandler);
}


CLogHandler::CLogHandler(std::string file) {

	m_FileName = file;
	m_FileHandler = fopen (m_FileName.c_str (), "w");
	if (0 == m_FileHandler)
		m_FileHandler = stdout;
}


void 
CLogHandler::log(std::string& message) {

	//FILE* fileHandler;

	//fileHandler = m_FileName == "" ? stdout : 0;

	//if (0 == fileHandler){
	//	
	//	fileHandler = fopen (m_FileName.c_str (), "a");
	//	if (0 == fileHandler){
	//		return;
	//	}
	//} 

	fwrite (message.c_str(), message.size (), 1, m_FileHandler);
	//if (fileHandler != stdout){
	//	fclose (fileHandler);
	//}
}


//void 
//CLogHandler::reset() {
//
//	if (m_FileHandler != stdout) 
//		m_FileHandler = fopen (m_FileName.c_str (), "w");
//	//FILE* fileHandler;
//
//	//fileHandler = m_FileName == "" ? stdout : 0;
//
//	//if (0 == fileHandler){
//	//	
//	//	fileHandler = fopen (m_FileName.c_str (), "w");
//	//	if (!fileHandler)
//	//		m_FileName = "";
//	//	else
//	//		fclose (fileHandler);
//	//} 
//}


void
CLogHandler::setFile(std::string &fileName) {

	if (m_FileHandler != stdout)
		fclose(m_FileHandler);

	m_FileName = fileName;
	m_FileHandler = fopen (m_FileName.c_str(), "w");
	if (0 == m_FileHandler)
		m_FileHandler = stdout;
}


void
CLogHandler::close() {

	if (m_FileHandler != stdout)
		fclose(m_FileHandler);
}