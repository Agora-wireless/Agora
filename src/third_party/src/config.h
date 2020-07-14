#pragma once
#include <stdlib.h>

namespace erpc {

// Pick a transport. This is hard to control from CMake.
class IBTransport;
class RawTransport;
class DpdkTransport;

#define CTransport IBTransport
static constexpr size_t kHeadroom = 40;
static constexpr size_t kIsRoCE = true;
}
