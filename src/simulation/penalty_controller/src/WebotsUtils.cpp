#include "WebotsUtils.hpp"
#include <cstdlib>
#include <unistd.h>

bool WaitForRobots(std::string &robotName, int wait)
{
    char *uri = getenv("WEBOTS_CONTROLLER_URL");
    char *user = getenv("USER");
    if (uri == nullptr || user == nullptr) {
        return false;
    }
    std::string uriStr(uri);
    std::string protocol = uriStr.substr(0, 3);
    if (protocol != "ipc") {
        return false;
    }
    int i = 6;
    while (i < uriStr.size() && uriStr[i] != '/') { ++i; }
    std::string port = uriStr.substr(6, i - 6);
    robotName = uriStr.substr(i + 1);
    std::string pipe = "/tmp/webots/" + std::string(user) + "/" + port + "/" + protocol + "/" + robotName;
    printf("process pipe: %s\n", pipe.c_str());
    bool ok = false;
    i = 0;
    while (!ok && i < wait) {
        ok = access(pipe.c_str(), F_OK) == 0;
        ++i;
        printf("waiting for: %s\n", pipe.c_str());
        usleep(1000000);
    }
    return ok;
}