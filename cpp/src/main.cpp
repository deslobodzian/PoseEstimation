// ZED includes

#include "Zed2.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;



int main(int argc, char **argv) {
    Zed2 zed;

    while (true) {
        auto pose = zed.getPose();
        zed.printPose(pose);
    }
    return 0;
}

