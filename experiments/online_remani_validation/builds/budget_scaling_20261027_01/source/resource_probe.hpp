#pragma once
// Experiment-only accounting. CLOCK_THREAD_CPUTIME_ID excludes scheduler waits.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace resource_probe {
inline double clockSeconds(clockid_t clock) {
  timespec t{}; clock_gettime(clock, &t); return t.tv_sec + t.tv_nsec * 1e-9;
}
inline double scheduledWait() {
  std::ifstream in("/proc/thread-self/schedstat");
  unsigned long long run=0, wait=0, slices=0; in >> run >> wait >> slices;
  return wait * 1e-9;
}
inline std::string& label() { static thread_local std::string s="normal"; return s; }
inline unsigned long long& sequence() { static thread_local unsigned long long n=0; return n; }
inline void write(const std::string& row) {
  const char* path=std::getenv("RESOURCE_TRACE_PATH");
  if(path) { std::ofstream f(path,std::ios::app); f << row << '\n'; }
}
struct Scope {
  std::string stage, extra, experiment_label;
  unsigned long long id;
  double wall, cpu, process_cpu, wait, sim;
  rusage usage{};
  bool success=false;
  Scope(const std::string& name):stage(name),experiment_label(label()),id(++sequence()),
    wall(clockSeconds(CLOCK_MONOTONIC)),cpu(clockSeconds(CLOCK_THREAD_CPUTIME_ID)),
    process_cpu(clockSeconds(CLOCK_PROCESS_CPUTIME_ID)),wait(scheduledWait()),sim(ros::Time::now().toSec()) {
    getrusage(RUSAGE_THREAD,&usage);
    std::ostringstream s; s << std::setprecision(16) << "{\"event\":\"begin\",\"stage\":\"" << stage
      << "\",\"label\":\"" << experiment_label << "\",\"id\":" << id << ",\"mono\":" << wall
      << ",\"sim\":" << sim << ",\"pid\":" << getpid() << ",\"tid\":" << syscall(SYS_gettid) << "}";
    write(s.str());
  }
  template<class T> void add(const std::string& key,const T& value) {
    std::ostringstream s; s << std::setprecision(16) << ",\"" << key << "\":" << value; extra += s.str();
  }
  ~Scope() {
    const double end=clockSeconds(CLOCK_MONOTONIC),end_cpu=clockSeconds(CLOCK_THREAD_CPUTIME_ID);
    const double end_process=clockSeconds(CLOCK_PROCESS_CPUTIME_ID),end_wait=scheduledWait();
    rusage finish{};getrusage(RUSAGE_THREAD,&finish);
    std::ostringstream s; s << std::setprecision(16) << "{\"event\":\"end\",\"stage\":\"" << stage
      << "\",\"label\":\"" << experiment_label << "\",\"id\":" << id << ",\"mono\":" << end
      << ",\"sim\":" << ros::Time::now().toSec() << ",\"pid\":" << getpid()
      << ",\"tid\":" << syscall(SYS_gettid) << ",\"wall_ms\":" << (end-wall)*1000
      << ",\"thread_cpu_ms\":" << (end_cpu-cpu)*1000
      << ",\"process_cpu_ms\":" << (end_process-process_cpu)*1000
      << ",\"runqueue_wait_ms\":" << (end_wait-wait)*1000
      << ",\"voluntary_switches\":" << finish.ru_nvcsw-usage.ru_nvcsw
      << ",\"involuntary_switches\":" << finish.ru_nivcsw-usage.ru_nivcsw
      << ",\"success\":" << (success?"true":"false") << extra << "}";
    write(s.str());
  }
};
inline void hashBytes(unsigned long long& h,const void* ptr,size_t n) {
  const auto* p=static_cast<const unsigned char*>(ptr);
  for(size_t i=0;i<n;++i) { h ^= p[i]; h *= 1099511628211ULL; }
}
template<class Container> void hashMatrices(unsigned long long& h,const Container& matrices) {
  for(const auto& m:matrices) {
    long rows=m.rows(),cols=m.cols();hashBytes(h,&rows,sizeof(rows));hashBytes(h,&cols,sizeof(cols));
    hashBytes(h,m.data(),m.size()*sizeof(double));
  }
}
} // namespace resource_probe
