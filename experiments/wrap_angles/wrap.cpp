#include <cmath>
#include <cstdio>

using namespace std;

int main() {
    for(double i = -15; i < 15; i += 0.1) {
        constexpr double two_pi = M_PI*2;

        printf("%5.5f\n", fmod(fmod(M_PI*i + M_PI, two_pi) + two_pi, two_pi) - M_PI);  // Slightly more accurate
        //printf("%5.5f\n", remainderf(M_PI*i, 2*M_PI));
    }

    return 0;
}
