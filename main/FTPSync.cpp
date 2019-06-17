#include "FTPSync.h"

FTPSync::FTPSync():
	m_ftp_client(&FTPSync::ftp_log_callback){}
 	

bool FTPSync::connect(const std::string& ftp_address,
			    	  uint16_t port,
			    	  const std::string& username,
			          const std::string& password,
			          const std::string& ftp_base_folder,
			          const std::string& local_folder)
{
	bool result = true;
	
	m_ftp_base_folder = ftp_base_folder;
	m_local_folder = local_folder;

	result = m_ftp_client.init();
	if (!result)
	{
		goto cleanup;
	}

	m_ftp_client.SetProgressFnCallback(NULL, &FTPSync::progress);

	result = m_ftp_client.InitSession(ftp_address,
		                            port,
		                            username,
		                            password);
	if (!result)
	{
		goto cleanup;
	}

	result = true;

cleanup:
	return result;
}

bool FTPSync::sync()
{
	bool result = false;
	std::string file_list;
	std::string segment;
	std::vector<std::string> seglist;

	/* Get file list from the server */

	result = m_ftp_client.List(m_ftp_base_folder, file_list);
	if (!result)
	{
		return result;
	}

	std::cout << file_list << std::endl;

	std::istringstream input;
    input.str(file_list);

	while(std::getline(input, segment, '\n'))
	{
	    if (segment == "." ||
	        segment == "..")
	    {
	    	continue;
	    }

	    std::string current_file_name = m_ftp_base_folder + segment;	
		std::cout << current_file_name << std::endl;
		std::string local_path = m_local_folder + segment;
		if (is_file_exist_in_local_folder(local_path))
		{
			std::cout << "File " << local_path << " already exist." << std::endl;
			continue;
		}

	    m_ftp_client.DownloadFile(local_path, current_file_name);
	}

	return result;
}


int FTPSync::progress(void *p,
                      double dltotal, double dlnow,
                      double ultotal, double ulnow)
{
	std::cout << dltotal / dlnow << "%" << std::endl;
	return 0;
}

bool FTPSync::is_file_exist_in_local_folder(const std::string& path)
{
	std::ifstream f(path.c_str());
    return f.good();
}

void FTPSync::ftp_log_callback(const std::string& output)
{
	std::cout << output << std::endl;
}

