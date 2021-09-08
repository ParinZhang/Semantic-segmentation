#include "timer.h"
#include <ratio>
#include <type_traits>


TimeCount::TimeCount()
{
	bEnd = false;
	promptStr = "elapsed: ";
	start_ = std::chrono::high_resolution_clock::now();
}

TimeCount::TimeCount(std::string prompt)
{
	bEnd = false;
	promptStr = prompt;
	start_ = std::chrono::high_resolution_clock::now();
}
void TimeCount::end()
{
	if (bEnd == false)
	{
		const auto end_ = std::chrono::high_resolution_clock::now();
		double dTimer = std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_).count();
		dTimer /= 1000000;
		std::cout << promptStr << ": " << dTimer << " s" << std::endl;
		bEnd = true;
	}
}

void TimeCount::temp_end(std::string prompt)
{
	if (bEnd == false)
	{
		const auto end_ = std::chrono::high_resolution_clock::now();
		double dTimer = std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_).count();
		dTimer /= 1000000;
		std::cout << prompt << ": " << dTimer << " s" << std::endl;
		start_ = end_;
	}
}

TimeCount::~TimeCount()
{
	if (bEnd == false)
	{
		const auto end_ = std::chrono::high_resolution_clock::now();
		double dTimer = std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_).count();
		dTimer /= 1000000;
		std::cout << promptStr << ": " << dTimer << " s" << std::endl;
	}
}





