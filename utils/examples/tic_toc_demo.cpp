// Main header to include
#include <utils/TicToc.h>

#include <thread>

int main(int argc, char *argv[])
{
  //Measuring time from default clock. This is a configurable number in the
  //header file as a macro, but is 0 as of now.
  std::cout << "sleeping for 2 sec and then printing the time slept for."
            << std::endl;
  tic;//resets the default clock and starts it.
  std::this_thread::sleep_for(std::chrono::duration<double>(2));
  toc; //stops the default clock
  std::cout << "printing time for default clock"  << std::endl;
  print_elapsed; // prints elapsed time in milliseconds.
  // retrieve elapsed time milliseconds, microseconds, seconds.
  std::cout << "milli: " << elapsed_msec << std::endl;
  std::cout << "micro: " << elapsed_usec << std::endl;
  std::cout << "sec: " << elapsed_sec << std::endl;
  // average elapsed is calculated as exponential moving average, which is also
  // used for frame rate computation 
  std::cout << "avg elapsed: " << avg_elapsed << std::endl;
  std::cout << "frame rate: " << tictoc_fps << std::endl;
  std::cout << "-------------------------------" << std::endl;

  // Multiple clocks can be used to measure time, this number is configurable as
  // well via a Macro in the header file, as of now 10 clocks (0-9) are
  // supported.
  std::cout << "sleeping for 2.5 sec and then printing the time slept for."
            << std::endl;
  ticn(1); //resets and starts clock 1
  std::this_thread::sleep_for(std::chrono::duration<double>(2.5));
  tocn(1); //stops clock 1
  print_elapsedn(1); //prints elapsed time for clock 1.
  // prints elapsed clock for default clock, this was stop when last toc was
  // called.
  print_elapsed;
  // retrieve elapsed time milliseconds, microseconds, seconds.
  std::cout << "milli[1]: " << elapsed_msecn(1) << std::endl;
  std::cout << "micro[1]: " << elapsed_usecn(1) << std::endl;
  std::cout << "sec[1]: " << elapsed_secn(1) << std::endl;
  // average elapsed is calculated as exponential moving average, which is also
  // used for frame rate computation 
  std::cout << "avg elapsed: " << avg_elapsedn(1) << std::endl;
  std::cout << "frame rate: " << tictoc_fpsn(1) << std::endl;
  std::cout << "-------------------------------" << std::endl;

  std::cout << "sleeping for 3.4 sec and then printing the time slept for."
            << std::endl;
  ticn(2); //resets and starts clock 2
  std::this_thread::sleep_for(std::chrono::duration<double>(3.4));
  tocn(2); //stops clock 2 
  print_elapsedn(2); //prints elapsed time for clock 2
  // prints elapsed clock for default clock, this was stop when last toc was
  // called.
  print_elapsed; 
  std::cout << "-------------------------------" << std::endl;

  //Nested clocks to measure times at different levels of code.
  std::cout
      << "sleeping for 1.5 and 4.5 sec and then printing the time slept for."
      << std::endl;
  tic;//resets the default clock and starts it.
  std::this_thread::sleep_for(std::chrono::duration<double>(1.5));
  ticn(1);
  std::this_thread::sleep_for(std::chrono::duration<double>(4.5));
  tocn(1);
  toc; //stops the default clock
  print_elapsed;
  print_elapsedn(1);
  std::cout << "-------------------------------" << std::endl;

  //Number of clocks right now is 10, anything below 0 and NUM_OF_TIMERS and
  //above is invalid.
  ticn(NUM_OF_TIMERS);
  tocn(NUM_OF_TIMERS);
  print_elapsedn(NUM_OF_TIMERS);

  ticn(-1);
  tocn(-1);
  print_elapsedn(-1);

  return 0;
}
