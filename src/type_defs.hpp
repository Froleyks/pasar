#pragma once

#include <limits>
#include <vector>

using variable_t = unsigned short int;
using value_t    = unsigned char;
using action_t   = unsigned int;
using State      = std::vector<value_t>;

// partial assignment
using Assignment = std::vector<std::pair<variable_t, value_t>>;

static constexpr value_t unassigned = std::numeric_limits<value_t>::max();
static constexpr action_t noop      = std::numeric_limits<action_t>::max();
