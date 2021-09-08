#include <chrono>
#include <iostream>

class TimeCount
{
  public:

    /**
    * @brief Default constructor
    */
     TimeCount();
     TimeCount(std::string prompt);
     ~TimeCount();  
     void end();
	 void temp_end(std::string prompt);

  private:

    bool bEnd;
    std::chrono::high_resolution_clock::time_point start_;
    std::string promptStr;
};
