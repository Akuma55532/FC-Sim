#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
namespace flight_core {
// Local process exclusion, scoped by ROS domain and MAVROS endpoint namespace.
// Not a distributed ownership protocol; remote publishers remain an operator concern.
class ControlLock {
public:
  explicit ControlLock(const std::string & endpoint) {
    const char * domain=std::getenv("ROS_DOMAIN_ID");
    const std::string key=std::string(domain?domain:"0")+":"+endpoint;
    const std::string path="/tmp/flight_core_"+std::to_string(getuid())+"_"+
      std::to_string(std::hash<std::string>{}(key))+".lock";
    fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (fd_<0) throw std::runtime_error("cannot open controller lock");
    if (flock(fd_,LOCK_EX|LOCK_NB)!=0) {
      close(fd_); fd_=-1;
      throw std::runtime_error("another flight_core controller owns this MAVROS namespace");
    }
  }
  ~ControlLock() {if(fd_>=0) close(fd_);}
  ControlLock(const ControlLock &)=delete;
  ControlLock & operator=(const ControlLock &)=delete;
private:
  int fd_{-1};
};
}
