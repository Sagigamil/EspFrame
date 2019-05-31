#ifndef FTP_SYNC_H
#define FTP_SYNC_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "FTPClient.h"

class FTPSync
{
public:
	FTPSync();

	bool connect(const std::string& ftp_address,
		    	 uint16_t port,
		    	 const std::string& username,
		         const std::string& password,
		         const std::string& ftp_base_folder,
		         const std::string& local_folder);

	bool sync();

private:
	static int progress(void *p,
              double dltotal, double dlnow,
              double ultotal, double ulnow);

	static bool is_file_exist_in_local_folder(const std::string& path);

	static void ftp_log_callback(const std::string& output);

	std::string m_ftp_address;
	uint16_t    m_port;

	std::string m_username;
	std::string m_password;
	
	std::string m_ftp_base_folder;
	std::string m_local_folder;

	CFTPClient m_ftp_client;
};

#endif /* FTP_SYNC_H */