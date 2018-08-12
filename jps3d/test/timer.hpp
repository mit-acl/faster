#include <chrono>

namespace JPS {
  class Timer {
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::milliseconds milliseconds;
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
    milliseconds Elapsed() const
    {
      return std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start);
    }
    private:
    high_resolution_clock::time_point _start;
  };
}
