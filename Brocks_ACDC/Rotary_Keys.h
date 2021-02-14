const int rotaryKeyVal[11] = { 580, 655, 725, 785, 825, 852, 875, 895, 910, 932, 947 };
const int NUM_KEYS = 12;

int getRotaryKey(unsigned int input) {
    int k;
    for (k = 0; k < NUM_KEYS; k++) {
        if (input < rotaryKeyVal[k]) {
            return k;
        }
    }
    if (k >= NUM_KEYS) k = 12;
    return k;
}