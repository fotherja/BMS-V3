#ifndef DVCC_h
#define DVCC_h

#include "Arduino.h"
#include "OpenBMS_James.h"

// ---- Tunables (LFP-ish defaults; adjust to your pack) ----
#define V_SOFT_HIGH_mV   3400   // start tapering charge above this
#define V_HARD_HIGH_mV   3500   // stop all charging at/above this
#define V_CLEAR_HIGH_mV  (V_HARD_HIGH_mV - 50)  // hysteresis to re-enable charge

#define V_SOFT_LOW_mV    2900   // start tapering discharge below this
#define V_HARD_LOW_mV    2750   // stop discharging at/below this
#define V_CLEAR_LOW_mV   (V_HARD_LOW_mV + 50)   // hysteresis to re-enable discharge

#define CVL_BASE_PC_mV   3350   // base per-cell CVL target in normal operation
#define CVL_MIN_PC_mV    3300   // don’t reduce CVL below this (soft floor)
#define CVL_HEADROOM_mV  75     // keep CVL at least pack+HEADROOM so charger doesn’t stall

#define MAX_CCL_mA       100000 // max charge current your pack allows (80 A example)
#define MAX_DCL_mA       180000 // max discharge current (150 A example)

#define CCL_SLEW_mA_per_s 10000 // how fast CCL is allowed to change
#define CVL_SLEW_mV_per_s 100   // how fast CVL is allowed to change

#define T_BLOCK_CHARGE_C  0     // Stop charging below this temperature
#define T_CCL_CLAMP_C     5     // Reduce charging current below this temperature
#define CCL_COLD_CAP_mA   15000 // Below T_CCL_CLAMP_C reduce charging current to this

// Proportional trim on CVL when soft-high is exceeded.
// Each extra mV above V_SOFT_HIGH reduces per-cell CVL by this many mV.
#define CVL_Kp_pc        2      // e.g. 2 mV PC trim per 1 mV over soft-high

static constexpr uint16_t CELL_ABS_MIN_mV = 2000;  // absolute floor for a valid LFP cell
static constexpr uint16_t CELL_ABS_MAX_mV = 3850;  // absolute ceiling (3.65 V + margin)

static constexpr uint32_t PACK_ABS_MIN_mV = 40000;
static constexpr uint32_t PACK_ABS_MAX_mV = 60000;

struct Telemetry {
  uint16_t maxCell_mV;
  uint16_t minCell_mV;
  uint32_t pack_mV;   
  float pack_temp;
};

struct Limits {
  uint32_t CVL_mV;
  uint32_t CCL_mA;
  uint32_t DCL_mA;
};

struct LimitsState {
  // Latches for hard limits (with hysteresis clear)
  bool chargeInhibit = false;
  bool dischargeInhibit = false;

  // Previous outputs for slew limiting
  uint32_t prevCVL_mV = NUM_OF_CELLS * CVL_BASE_PC_mV;
  uint32_t prevCCL_mA = 0;
  uint32_t prevDCL_mA = MAX_DCL_mA;
};

// Function prototypes:
void computeLimits(const Telemetry& t, LimitsState& s, Limits& out, float dt_s);
void toVictronScaling(const Limits& in, uint16_t& CVL_Vx10, uint16_t& CCL_Ax10, uint16_t& DCL_Ax10);

#endif


