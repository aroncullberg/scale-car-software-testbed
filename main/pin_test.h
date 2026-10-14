#pragma once

namespace pin_test {

// Walks every usable GPIO on this chip and reports whether its input buffer
// and output driver still work. Needs nothing attached to the board. Run it
// before anything else claims a pin, since each pad is left reset afterwards.
void run_all();

} // namespace pin_test
