#include <math.h>

float pitch2freq(float pitch) {
    return exp((pitch + 36.37631656229591) * 0.0577622650466621); 
}

float freq2pitch(float f) {
    return log(f) * 17.312340490667562 - 36.37631656229591;
}
