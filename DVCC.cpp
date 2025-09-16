#include "DVCC.h"

// helper clamp
template <typename T>
static inline T clampT(T v, T lo, T hi) { return std::max(lo, std::min(v, hi)); }

// Slew limit utility: move prev toward target by at most rate*dt_s (abs)
uint32_t slewToward(uint32_t prev, uint32_t target, uint32_t rate_per_s, float dt_s) {
  if (prev == target || rate_per_s == 0) return target;
  int64_t diff = (int64_t)target - (int64_t)prev;
  int64_t maxStep = (int64_t)(rate_per_s * dt_s);
  if (diff > 0) diff = std::min<int64_t>(diff, maxStep);
  else          diff = std::max<int64_t>(diff, -maxStep);
  return (uint32_t)((int64_t)prev + diff);
}

// Main function: compute new CVL/CCL/DCL
void computeLimits(const Telemetry& t, LimitsState& s, Limits& out, float dt_s)
{
  // ---- Update latches with hysteresis ----
  if (!s.chargeInhibit && t.maxCell_mV >= V_HARD_HIGH_mV) s.chargeInhibit = true;
  if (s.chargeInhibit  && t.maxCell_mV <= V_CLEAR_HIGH_mV) s.chargeInhibit = false;

  if (!s.dischargeInhibit && t.minCell_mV <= V_HARD_LOW_mV) s.dischargeInhibit = true;
  if (s.dischargeInhibit  && t.minCell_mV >= V_CLEAR_LOW_mV) s.dischargeInhibit = false;

  // ---- CVL target (pack mV) ----
  // Base CVL is per-cell target times N cells
  uint32_t baseCVL_mV = (uint32_t)NUM_OF_CELLS * (uint32_t)CVL_BASE_PC_mV;

  // Proportional trim when above soft-high
  int32_t over_mV = (int32_t)t.maxCell_mV - (int32_t)V_SOFT_HIGH_mV;
  int32_t trim_pc_mV = (over_mV > 0) ? (over_mV * CVL_Kp_pc) : 0;

  uint32_t cvl_pc_mV = clampT<int32_t>((int32_t)CVL_BASE_PC_mV - trim_pc_mV, (int32_t)CVL_MIN_PC_mV, (int32_t)CVL_BASE_PC_mV);
  uint32_t cvl_target_mV = (uint32_t)NUM_OF_CELLS * cvl_pc_mV;

  // Keep CVL at least a little above present pack voltage to avoid stalling/oscillation
  if (cvl_target_mV < t.pack_mV + CVL_HEADROOM_mV) {
    cvl_target_mV = t.pack_mV + CVL_HEADROOM_mV;
  }

  // If hard-high is latched, freeze CVL to (≈) current pack voltage to stop any further rise
  if (s.chargeInhibit) {
    cvl_target_mV = t.pack_mV; // (headroom not applied when we’re inhibiting charge)
  }

  // Slew-limit CVL
  out.CVL_mV = slewToward(s.prevCVL_mV, cvl_target_mV, CVL_SLEW_mV_per_s, dt_s);
  s.prevCVL_mV = out.CVL_mV;

  // ---- CCL (charge current) taper ----
  uint32_t ccl_target_mA = MAX_CCL_mA;
  if (s.chargeInhibit) {
    ccl_target_mA = 0;
  } else if (t.maxCell_mV > V_SOFT_HIGH_mV) {
    // Linear ramp from MAX_CCL at soft-high down to 0 at hard-high
    // fraction = (V_HARD_HIGH - Vmax) / (V_HARD_HIGH - V_SOFT_HIGH)
    int32_t num = (int32_t)V_HARD_HIGH_mV - (int32_t)t.maxCell_mV;
    int32_t den = (int32_t)V_HARD_HIGH_mV - (int32_t)V_SOFT_HIGH_mV;
    float frac = clampT<float>((float)num / (float)den, 0.0f, 1.0f);
    ccl_target_mA = (uint32_t)(frac * (float)MAX_CCL_mA);
  }

  // Temperature-based CCL
  if (t.pack_temp <= T_BLOCK_CHARGE_C) {
    ccl_target_mA = 0;
  }
  else if (t.pack_temp < T_CCL_CLAMP_C) {
    ccl_target_mA = clampT<uint32_t>(ccl_target_mA, 0u, CCL_COLD_CAP_mA);
  }

  // Slew-limit CCL
  out.CCL_mA = slewToward(s.prevCCL_mA, ccl_target_mA, CCL_SLEW_mA_per_s, dt_s);
  s.prevCCL_mA = out.CCL_mA;

  // ---- DCL (discharge current) taper ----
  uint32_t dcl_target_mA = MAX_DCL_mA;
  if (s.dischargeInhibit) {
    dcl_target_mA = 0;
  } else if (t.minCell_mV < V_SOFT_LOW_mV) {
    // Linear ramp from MAX_DCL at soft-low down to 0 at hard-low
    int32_t num = (int32_t)t.minCell_mV - (int32_t)V_HARD_LOW_mV;
    int32_t den = (int32_t)V_SOFT_LOW_mV - (int32_t)V_HARD_LOW_mV;
    float frac = clampT<float>((float)num / (float)den, 0.0f, 1.0f);
    dcl_target_mA = (uint32_t)(frac * (float)MAX_DCL_mA);
  }
  // (Optional) Slew-limit DCL up as well if you want symmetry; here we leave it immediate.
  out.DCL_mA = dcl_target_mA;
  s.prevDCL_mA = out.DCL_mA;
}

// ---- Helper to convert to Victron DVCC scaling (for 0x351) ----
// CVL: V×10  -> uint16
// CCL/DCL: A×10 -> uint16 (positive numbers)
void toVictronScaling(const Limits& in, uint16_t& CVL_Vx10, uint16_t& CCL_Ax10, uint16_t& DCL_Ax10)
{
  // mV -> V×10: divide by 100 mV
  CVL_Vx10 = (uint16_t)clampT<uint32_t>((in.CVL_mV + 50) / 100, 0, 65535);     // round
  // mA -> A×10: divide by 100 mA
  CCL_Ax10 = (uint16_t)clampT<uint32_t>((in.CCL_mA + 50) / 100, 0, 65535);
  DCL_Ax10 = (uint16_t)clampT<uint32_t>((in.DCL_mA + 50) / 100, 0, 65535);
}


















