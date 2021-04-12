#pragma once

#include "types.hpp"

class TemporalAligner {
    virtual state_t align(float delta_t) = 0;
};
