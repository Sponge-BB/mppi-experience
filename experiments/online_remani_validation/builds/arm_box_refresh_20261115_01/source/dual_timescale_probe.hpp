#pragma once
// Experiment-only, coarse event accounting. No objective/EDT leaf timers.
#include <ros/ros.h>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <time.h>
#include <unordered_map>
#include <unistd.h>

namespace dual_probe {
inline double mono() { timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9; }
inline std::string quote(const std::string& s) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:s) {if(c=='"'||c=='\\')out<<'\\'<<char(c);else if(c=='\n')out<<"\\n";
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<char(c);}
  return out.str()+'"';
}
struct Fields {
  std::string text;
  template<class T> Fields& add(const std::string& k,const T& value) {
    std::ostringstream s;s<<std::setprecision(17)<<','<<quote(k)<<':'<<value;text+=s.str();return *this;
  }
  Fields& str(const std::string& k,const std::string& value) {text+=','+quote(k)+':'+quote(value);return *this;}
};
inline unsigned long long& request() {static thread_local unsigned long long id=0;return id;}
inline unsigned long long nextRequest() {static std::atomic<unsigned long long> serial{0};return ++serial;}
inline void emit(const std::string& event,const Fields& fields=Fields()) {
  const char* path=std::getenv("DUAL_TIMELINE_PATH");if(!path)return;
  std::ostringstream s;s<<std::setprecision(17)<<"{\"event\":"<<quote(event)<<",\"mono\":"<<mono()
    <<",\"sim\":"<<ros::Time::now().toSec()<<",\"pid\":"<<getpid()<<",\"request_id\":"<<request()<<fields.text<<"}\n";
  // A single O_APPEND write per coarse event; no clock calls inside cost loops.
  static std::mutex lock;std::lock_guard<std::mutex> guard(lock);
  const int fd=open(path,O_WRONLY|O_CREAT|O_APPEND,0600);if(fd<0)return;
  const std::string row=s.str();const auto ignored=write(fd,row.data(),row.size());(void)ignored;close(fd);
}
struct Scope {
  std::string stage;Fields fields;double start;bool success=false,finished=false;
  explicit Scope(const std::string& name,const Fields& f=Fields()):stage(name),fields(f),start(mono()) {
    Fields begin=fields;begin.str("stage",stage);emit("stage_begin",begin);
  }
  void finish() {if(finished)return;finished=true;fields.str("stage",stage).add("wall_ms",(mono()-start)*1000).add("success",success?"true":"false");emit("stage_end",fields);}
  ~Scope() {finish();}
};
struct Request {
  unsigned long long previous;double start;Fields fields;bool success=false;
  explicit Request(const Fields& f):previous(request()),start(mono()),fields(f) {
    request()=nextRequest();emit("request_begin",fields);
  }
  ~Request() {fields.add("wall_ms",(mono()-start)*1000).add("success",success?"true":"false");emit("request_end",fields);request()=previous;}
};
inline void condition(const std::string& key,bool waiting,const Fields& fields=Fields()) {
  static std::unordered_map<std::string,double> waits;
  auto it=waits.find(key);
  if(waiting&&it==waits.end()) {waits[key]=mono();Fields f=fields;f.str("wait_key",key);emit("wait_begin",f);}
  if(!waiting&&it!=waits.end()) {Fields f=fields;f.str("wait_key",key).add("wall_ms",(mono()-it->second)*1000);emit("wait_end",f);waits.erase(it);}
}
template<class T> unsigned long long hash(const T& s) {
  unsigned long long h=1469598103934665603ULL;for(unsigned char c:s){h^=c;h*=1099511628211ULL;}return h;
}
} // namespace dual_probe
