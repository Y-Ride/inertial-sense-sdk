#include <gtest/gtest.h>

#if 1 // ((ROS_VERSION_MAJOR == 1) && (ROS_VERSION_MINOR <= 13))
namespace testing
{
    namespace internal
    {
#if defined(ROS_DISTRO_JAZZY) // We are using a new-enough version that these are already defined...
        enum GTestColor {
            COLOR_DEFAULT,
            COLOR_RED,
            COLOR_GREEN,
            COLOR_YELLOW
        };
#endif

        extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
    }
}
#define PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while(0)
#else
#define PRINTF(...)
#endif

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout()
    {
        printf("%s",str().c_str());
    }
};

#define TEST_COUT  TestCout()
