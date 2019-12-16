#include <chrono>

namespace JPS
{
class Timer
{
  typedef std::chrono::high_resolution_clock high_resolution_clock;
  typedef std::chrono::milliseconds milliseconds;
  // typedef std::chrono::microseconds microseconds;

public:
  explicit Timer(bool run = false)
  {
    if (run)
      Reset();
  }
  void Reset()
  {
    _start = high_resolution_clock::now();
  }
  double ElapsedMs() const
  {
    return (std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start)).count();
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
  {
    return out << timer.ElapsedMs();
  }

private:
  high_resolution_clock::time_point _start;
};

/*class ROSTimer
{
public:
  ROSTimer(bool run = false)
  {
    if (run)
      Reset();
  }
  void Reset()
  {
    _start = ros::Time::now().toSec();
  }
  double ElapsedMs() const
  {
    return 1000 * (ros::Time::now().toSec() - _start);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSTimer& timer)
  {
    return out << timer.ElapsedMs();
  }

private:
  double _start;
};

class ROSWallTimer
{
public:
  ROSWallTimer(bool run = false)
  {
    if (run)
      Reset();
  }
  void Reset()
  {
    _start = ros::WallTime::now().toSec();
    std::cout << "Starting timer " << _start << std::endl;
  }
  double ElapsedMs() const
  {
    std::cout << "En elapsed" << std::endl;
    std::cout << "ros::Time::now().toSec()" << ros::Time::now().toSec() << std::endl;
    std::cout << "_start" << _start << std::endl;
    return 1000 * (ros::WallTime::now().toSec() - _start);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSWallTimer& timer)
  {
    return out << timer.ElapsedMs();
  }

private:
  double _start;
};*/
}  // namespace JPS
