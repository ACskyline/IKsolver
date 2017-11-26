#include <windows.h>
#include <vector>
#include <string>
#include <Shlobj.h>
#include <Shobjidl.h>

#ifdef UNICODE
std::wstring s2ws(const std::string& s)
{
 int len;
 int slength = (int)s.length() + 1;
 len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
 wchar_t* buf = new wchar_t[len];
 MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
 std::wstring r(buf);
 delete[] buf;
 return r;
}

std::string ws2s(const std::wstring &wstr)
{
LPSTR szText = new CHAR[wstr.size()+1];
WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), -1, szText, (int)wstr.size()+1, 0, 0);
std::string tmp = szText;
delete[] szText;
return tmp;
}

std::string PromptToLoadMotion()
{
	OPENFILENAME ofn;

	wchar_t filename[1024];
	memset(filename, 0, sizeof(filename));

	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = TEXT("AMC Files\0*.amc\0BVH Files\0*.bvh\0All Files\0*.*\0\0");
    ofn.lpstrFile = filename; //(LPWSTR) 
	ofn.nMaxFile = 1024;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;

	BOOL result = GetOpenFileName(&ofn);
	if(result)
	{
        //for (int i = 0; i < 2014; i++) printf("%s ", filename[i]);
        std::string result = ws2s(filename);
        return result;
	}
	else
	{
		DWORD err = CommDlgExtendedError();
		err = err;
	}
    return "";
}

std::vector<std::string> GetFilenamesInDir(const std::string& dirname)
{
    printf("Target directory is %s\n", dirname.c_str());
    std::string dirnamestr = dirname;
    dirnamestr += "\\*";

    WIN32_FIND_DATA ffd;
    HANDLE hFind = INVALID_HANDLE_VALUE;
    LARGE_INTEGER filesize;

    hFind = FindFirstFile((LPCWSTR) s2ws(dirnamestr).c_str(), &ffd);
    if (INVALID_HANDLE_VALUE == hFind) 
    {
       printf("Cannot open drectory: %s\n", dirname);
       return std::vector<std::string>();
    }
   
    std::vector<std::string> names;
    // List all the files in the directory with some info about them.
    do
    {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      {
         continue;
      }
      else
      {
         filesize.LowPart = ffd.nFileSizeLow;
         filesize.HighPart = ffd.nFileSizeHigh;
         std::string filename = ws2s(ffd.cFileName);
         printf("  %s   %ld bytes\n", filename.c_str(), filesize.QuadPart);
         names.push_back(filename);
      }
    }
    while (FindNextFile(hFind, &ffd) != 0);
 
    FindClose(hFind);
    return names;
}

#else
std::string PromptToLoad(const std::string& filetypes)
{
	OPENFILENAME ofn;
	char filename[1024];
	memset(filename, 0, sizeof(filename));

	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = (LPCSTR) filetypes.c_str();
    ofn.lpstrFile = filename;
	ofn.nMaxFile = 1024;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;

	BOOL result = GetOpenFileName(&ofn);
	if(result)
	{
        return filename;
	}
	else
	{
		DWORD err = CommDlgExtendedError();
		err = err;
	}
    return "";
}

static int CALLBACK BrowseCallbackProc(HWND hwnd,UINT uMsg, LPARAM lParam, LPARAM lpData)
{
	// If the BFFM_INITIALIZED message is received
	// set the path to the start path.
	switch (uMsg)
	{
		case BFFM_INITIALIZED:
		{
			if (NULL != lpData)
			{
				SendMessage(hwnd, BFFM_SETSELECTION, TRUE, lpData);
			}
		}
	}
	return 0; // The function should always return 0.
}

std::string PromptToLoadDir()
{
    char cwd[1024];
    GetCurrentDirectory(1024, cwd);

	BROWSEINFO bi;
	TCHAR Buffer[MAX_PATH];
	ZeroMemory(Buffer, MAX_PATH);
	ZeroMemory(&bi, sizeof(bi));
	bi.hwndOwner = NULL;
	bi.pszDisplayName = Buffer;
	bi.lpszTitle = "";
	bi.ulFlags = BIF_NEWDIALOGSTYLE | BIF_RETURNONLYFSDIRS | BIF_SHAREABLE;
	bi.lpfn = BrowseCallbackProc;
    bi.lParam = (LPARAM) cwd;

    LPCITEMIDLIST pFolder = SHBrowseForFolder(&bi);
	if(pFolder)
	{
        SHGetPathFromIDList(pFolder, Buffer );
        std::string filename = Buffer;
        return filename;
	}
	else
	{
		DWORD err = CommDlgExtendedError();
		err = err;
	}
    return "";
}

std::vector<std::string> GetFilenamesInDir(const std::string& dirname)
{
    printf("Target directory is %s\n", dirname.c_str());
    std::string dirnamestr = dirname;
    dirnamestr += "\\*";

    WIN32_FIND_DATA ffd;
    HANDLE hFind = INVALID_HANDLE_VALUE;
    LARGE_INTEGER filesize;

    hFind = FindFirstFile(dirnamestr.c_str(), &ffd);
    if (INVALID_HANDLE_VALUE == hFind) 
    {
       printf("Cannot open drectory: %s\n", dirname);
       return std::vector<std::string>();
    }
   
    std::vector<std::string> names;
    // List all the files in the directory with some info about them.
    do
    {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      {
         continue;
      }
      else
      {
         filesize.LowPart = ffd.nFileSizeLow;
         filesize.HighPart = ffd.nFileSizeHigh;
         std::string filename = ffd.cFileName;
         //printf("  %s   %ld bytes\n", filename.c_str(), filesize.QuadPart);
         names.push_back(filename);
      }
    }
    while (FindNextFile(hFind, &ffd) != 0);
 
    FindClose(hFind);
    return names;
}

#endif
