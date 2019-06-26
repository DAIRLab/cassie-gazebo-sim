#pragma once

#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_cassie_in.hpp"

namespace dairlib {

// Convert from an LCM message, dairlib::lcmt_cassie_out, to the Agility
// cassie_out_t struct
void cassieOutFromLcm(const lcmt_cassie_out& message,
    cassie_out_t* cassie_out);

// Convert from Agility cassie_out_t struct to LCM message,
// dairlib::lcmt_cassie_out. Since the struct does not include time, time (s)
// is a required additional input
void cassieOutToLcm(const cassie_out_t& cassie_out, double time_seconds,
    lcmt_cassie_out* message);

// Convert from Agility cassie_user_in_t struct to LCM message,
// dairlib::lcmt_cassie_in. Since the struct does not include time, time (s)
// is a required additional input
void cassieInToLcm(const cassie_user_in_t& cassie_in, double time_seconds,
    lcmt_cassie_in* message);

// Convert from an LCM message, dairlib::lcmt_cassie_in to
// Agility cassie_user_in_t struct
void cassieInFromLcm(const lcmt_cassie_in& message,
    cassie_user_in_t* cassie_in);

}  // namespace dairlib
