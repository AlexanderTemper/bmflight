#include "sput.h"
#include "common/debug.h"
#include "global.h"

uint8_t debugDataArray[12];
static void debugDataImpl(const uint8_t* data, uint16_t len) {
    for (int i = 0; i < len; i++) {
        debugDataArray[i] = *data;
        data++;
    }
}

static bool checkArray(const uint8_t* src, const uint8_t* des, uint16_t len) {
    for (int i = 0; i < len; i++) {
        char ist = *src++;
        char soll = *des++;
        if (ist != soll) {
            printf("%c !== %c\n", ist, soll);
            return false;
        }
    }
    return true;
}

static void test_printDebug(void) {

    initDebug(&debugDataImpl);
    printDebug("test");
    uint8_t soll[12] = { 't', 'e', 's', 't', 0, 0, 0, 0, 0, 0, 0, 0 };
    sput_fail_unless(checkArray(debugDataArray, soll, 12), "print test");

    memset(debugDataArray, 0, 12);

    printInt16Debug(-12345);
    uint8_t soll2[12] = { '-', '1', '2', '3', '4', '5', 0, 0, 0, 0, 0, 0 };
    sput_fail_unless(checkArray(debugDataArray, soll2, 12), "int Print");

    memset(debugDataArray, 0, 12);
    printInt32Debug(-1234567890);
    uint8_t soll3[12] = { '-', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 0 };
    sput_fail_unless(checkArray(debugDataArray, soll3, 12), "int Print");
    //static  = { 4, MSP_DEBUGMSG, 't', 'e', 's', 't' };

}

int main(int argc, char *argv[]) {
    sput_start_testing();

    sput_enter_suite("Test Debbug");
    sput_run_test(test_printDebug);
    sput_finish_testing();

    return sput_get_return_value();
}
