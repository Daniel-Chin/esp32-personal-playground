#include <stdio.h>

void printArray(uint8_t* array, int len) {
    printf("[");
    for (int i = 0; i < len; i ++) {
        printf("%d", array[i]);
        printf(", ");
    }
    printf("]");
}
