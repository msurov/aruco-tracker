#ifdef _MSC_VER
    #include <Windows.h>
    #include <locale>
    #include <codecvt>

#else // GCC
    #include <dirent.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <unistd.h>
#endif 

#include <algorithm>
#include <string>
#include <cctype>
#include <signal.h>
#include "traces.h"
#include "common.h"

using namespace std;

vector<double> json_arr_to_vec(jsonxx::Array const& arr)
{
    vector<double> res(arr.size());

    for (int i = 0; i < (int)arr.size(); ++ i)
        res[i] = arr.get<jsonxx::Number>(i);

    return res;
}

int64_t __beginning_epoch_usec()
{
    static const int64_t t0 = __epoch_usec();
    return t0;
}

/*
 * pathes
 */

tuple<string, string> splitname(string const& path)
{
    auto n = path.find_last_of("/\\");
    if (n == string::npos)
        return make_tuple("", path);
    n += 1;
    return make_tuple(path.substr(0, n), path.substr(n));
}

tuple<string, string> splitext(string const& path)
{
    auto n = path.find_last_of("/\\.");
    if (n == string::npos)
        return make_tuple(path, "");

    if (path[n] == '/' || path[n] == '\\')
        return make_tuple(path, "");

    return make_tuple(path.substr(0, n), path.substr(n));
}

string getname(string const& path)
{
    auto n = path.find_last_of("/\\");
    if (n == string::npos)
        return "";
    return path.substr(n + 1);
}

string getext(string const& path)
{
    auto n = path.find_last_of("/\\.");
    if (n == string::npos)
        return "";

    if (path[n] == '/' || path[n] == '\\')
        return "";

    return path.substr(n);
}

string stripstr(string const& s)
{
    size_t e1 = s.find_first_not_of(" \t\n\r");
    if (e1 == string::npos)
        return "";

    size_t e2 = s.find_last_not_of(" \t\n\r");
    return s.substr(e1, e2 - e1 + 1);
}

string tolower(string const& s)
{
    string result = s;
    // auto f = [](char c) { std::tolower(c); };
    // std::transform(s.begin(), s.end(), result.begin(), f);

    for (auto& e : result)
        e = std::tolower(e);

    return result;
}

int compare_case_insensitive(string const& s1, string const& s2)
{
    return tolower(s1).compare(tolower(s2));
}

std::string get_digits_substr(std::string const& s)
{
    tuple<int, int> lims(0,0);

    for (int i = 0; i < s.size(); ++i)
    {
        if (isdigit(s[i]))
        {
            int start = i;
            while (i < s.size() && isdigit(s[i]))
                ++i;
            int end = i;

            if (get<1>(lims) - get<0>(lims) < end - start)
                lims = make_tuple(start, end);
        }
    }

    return s.substr(get<0>(lims), get<1>(lims));
}

bool endswith(string const& str, string const& ending)
{
    if (str.size() < ending.size())
        return false;

    return str.compare(str.size() - ending.size(), ending.size(), ending) == 0;
}

static weak_ptr<signal_handler_t> ptr_sigint_handler;
static weak_ptr<signal_handler_t> ptr_sigterm_handler;

void sigint_handler(int sig)
{
    if (sig == SIGINT)
    {
        dbg_msg("sigint received");
        auto f = ptr_sigint_handler.lock();

        if (f)
        {
            dbg_msg("sigint handler called");
            (*f)();
        }
        else
        {
            dbg_msg("no sigint handler");
        }
    }
}

void sigterm_handler(int sig)
{
    if (sig == SIGTERM)
    {
        dbg_msg("sigterm received");
        auto f = ptr_sigterm_handler.lock();

        if (f)
        {
            dbg_msg("sigterm handler called");
            (*f)();
        }
        else
        {
            dbg_msg("no sigterm handler");
        }
    }
}

void set_sigint_handler(shared_ptr<signal_handler_t> ptr_handler)
{
    signal(SIGINT, sigint_handler);
    ptr_sigint_handler = ptr_handler;
}

void set_sigterm_handler(std::shared_ptr<signal_handler_t> ptr_handler)
{
    signal(SIGTERM, sigterm_handler);
    ptr_sigterm_handler = ptr_handler;
}

bool __match_mask(string const& name, string const& mask, int iname, int imask)
{
    while (true)
    {
        if (iname == name.size())
            break;

        if (imask == mask.size())
            return false;

        char c = mask[imask];

        switch (c)
        {
        case '*':
            for (int iname1 = iname; iname1 <= name.size(); ++ iname1)
            {
                if (__match_mask(name, mask, iname1, imask + 1))
                    return true;
            }
            return false;
        case '?':
            ++ iname;
            ++ imask;
            break;
        default:
            if (c != name[iname])
                return false;
            ++ iname;
            ++ imask;
            break;
        }
    }

    if (imask == mask.size())
        return true;

    return false;
}

bool match_mask(string const& filename, string const& mask)
{
    return __match_mask(filename, mask, 0, 0);
}


#if WIN32

std::vector<std::string> get_files(std::string const& mask)
{
    vector<string> files;

    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    std::wstring wmask = converter.from_bytes(mask);

    WIN32_FIND_DATAW data;
    HANDLE h = FindFirstFileW(wmask.c_str(), &data);

    if (h == INVALID_HANDLE_VALUE)
        return files;

    string dirpath;
    tie(dirpath, ignore) = splitname(mask);

    do
    {
        if (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            continue;

        auto filepath = dirpath + converter.to_bytes(data.cFileName);
        files.push_back(filepath);
    }
    while (FindNextFileW(h, &data));

    FindClose(h);
    return files;
}

#elif defined(__unix__)

std::vector<std::string> get_files(std::string const& mask)
{
    vector<string> files;

    string dirpath;
    string namemask;
    tie(dirpath, namemask) = splitname(mask);

    DIR* d = opendir(dirpath.c_str());
    if (!d)
        throw runtime_error("can't open dir '" + dirpath + "'");

    do
    {
        dirent entry, *pentry;
        int status;

        status = readdir_r(d, &entry, &pentry);

        if (status)
            break;

        if (!pentry)
            break;

        if (!match_mask(pentry->d_name, namemask))
            continue;

        auto filepath = dirpath + string(pentry->d_name);

        struct stat s;
        status = stat(filepath.c_str(), &s);
        if (status)
            continue;

        if (S_ISREG(s.st_mode))
        {
            files.push_back(filepath);
        }
    } 
    while (true);
    closedir(d);

    return files;
}

#endif
