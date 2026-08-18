#pragma once
#include "HWT906_Def.h"

class HWT906_Parser{
  public:
    IMUData data;

    HWT906_Parser();

    bool parseByte(uint8_t b);

    void reset();
  
  private:
    enum class State {
        WAIT_HEADER,
        WAIT_TYPE,
        RECEIVE_DATA
    };

    State state = State::WAIT_HEADER;
    uint8_t buffer[11];
    uint8_t count = 0;

    bool verifyChecksum();
    void decode();
};