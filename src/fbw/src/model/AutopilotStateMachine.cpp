#include "AutopilotStateMachine.h"
#include "AutopilotStateMachine_private.h"
#include "Double2MultiWord.h"
#include "MultiWordIor.h"
#include "rt_remd.h"
#include "uMultiWord2Double.h"

const uint8_T AutopilotStateMachine_IN_FLARE = 1U;
const uint8_T AutopilotStateMachine_IN_GA_TRK = 1U;
const uint8_T AutopilotStateMachine_IN_HDG = 1U;
const uint8_T AutopilotStateMachine_IN_LAND = 2U;
const uint8_T AutopilotStateMachine_IN_LOC = 2U;
const uint8_T AutopilotStateMachine_IN_LOC_CPT = 3U;
const uint8_T AutopilotStateMachine_IN_LOC_TRACK = 4U;
const uint8_T AutopilotStateMachine_IN_NAV = 3U;
const uint8_T AutopilotStateMachine_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T AutopilotStateMachine_IN_OFF = 2U;
const uint8_T AutopilotStateMachine_IN_ON = 3U;
const uint8_T AutopilotStateMachine_IN_ROLL_OUT = 5U;
const uint8_T AutopilotStateMachine_IN_RWY = 4U;
const uint8_T AutopilotStateMachine_IN_RWY_TRK = 5U;
const uint8_T AutopilotStateMachine_IN_InAir = 1U;
const uint8_T AutopilotStateMachine_IN_OnGround = 2U;
const uint8_T AutopilotStateMachine_IN_ALT = 1U;
const uint8_T AutopilotStateMachine_IN_ALT_CPT = 2U;
const uint8_T AutopilotStateMachine_IN_ALT_CST = 3U;
const uint8_T AutopilotStateMachine_IN_ALT_CST_CPT = 4U;
const uint8_T AutopilotStateMachine_IN_CLB = 5U;
const uint8_T AutopilotStateMachine_IN_DES = 6U;
const uint8_T AutopilotStateMachine_IN_GS = 7U;
const uint8_T AutopilotStateMachine_IN_GS_CPT = 2U;
const uint8_T AutopilotStateMachine_IN_GS_TRACK = 3U;
const uint8_T AutopilotStateMachine_IN_LAND_k = 4U;
const uint8_T AutopilotStateMachine_IN_OFF_o = 1U;
const uint8_T AutopilotStateMachine_IN_ON_p = 2U;
const uint8_T AutopilotStateMachine_IN_OP_CLB = 8U;
const uint8_T AutopilotStateMachine_IN_OP_DES = 9U;
const uint8_T AutopilotStateMachine_IN_SRS = 10U;
const uint8_T AutopilotStateMachine_IN_SRS_GA = 3U;
const uint8_T AutopilotStateMachine_IN_VS = 11U;
void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift1(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 1);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_X_TO_OFF(const ap_sm_output *BusAssignment)
{
  return ((!BusAssignment->input.FD_active) && (BusAssignment->output.enabled_AP1 == 0.0) &&
          (BusAssignment->output.enabled_AP2 == 0.0)) || (BusAssignment->data.flight_phase == 7.0) ||
    ((BusAssignment->data.flight_phase == 0.0) && (BusAssignment->data.throttle_lever_1_pos < 35.0) &&
     (BusAssignment->data.throttle_lever_2_pos < 35.0));
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_X_TO_GA_TRK(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.condition.GA_TRACK;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_HDG(const ap_sm_output *BusAssignment)
{
  return BusAssignment->input.HDG_pull;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_NAV(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.condition.NAV && (BusAssignment->lateral.armed.NAV || BusAssignment->input.HDG_push ||
    BusAssignment->input.DIR_TO_trigger);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_NAV_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_NAV;
  AutopilotStateMachine_B.out_g.law = lateral_law_HPATH;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_HDG_entry(const ap_sm_output *BusAssignment)
{
  if (BusAssignment->input.TRK_FPA_mode) {
    AutopilotStateMachine_B.out_g.mode = lateral_mode_TRACK;
    AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  } else {
    AutopilotStateMachine_B.out_g.mode = lateral_mode_HDG;
    AutopilotStateMachine_B.out_g.law = lateral_law_HDG;
  }
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_LOC(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.armed.LOC && BusAssignment->lateral.condition.LOC_CPT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_HDG_during(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->input.Psi_fcu_deg;
  if (BusAssignment->input.TRK_FPA_mode) {
    AutopilotStateMachine_B.out_g.mode = lateral_mode_TRACK;
    AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  } else {
    AutopilotStateMachine_B.out_g.mode = lateral_mode_HDG;
    AutopilotStateMachine_B.out_g.law = lateral_law_HDG;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LOC_CPT_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LOC_CPT;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_CPT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_NONE;
  AutopilotStateMachine_B.out_g.law = lateral_law_NONE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ROLL_OUT_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_ROLL_OUT;
  AutopilotStateMachine_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_FLARE_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_FLARE;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_LOC_TO_X(const ap_sm_output *BusAssignment)
{
  boolean_T isGsArmedOrActive;
  isGsArmedOrActive = (BusAssignment->vertical_previous.armed.GS || (BusAssignment->vertical_previous.output.mode ==
    vertical_mode_GS_CPT) || (BusAssignment->vertical_previous.output.mode == vertical_mode_GS_TRACK));
  return (BusAssignment->input.LOC_push && (!isGsArmedOrActive)) || (BusAssignment->input.APPR_push && isGsArmedOrActive);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LOC_TRACK_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LOC_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LAND_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LAND;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_NAV_TO_HDG(const ap_sm_output *BusAssignment)
{
  return BusAssignment->input.HDG_pull || (!BusAssignment->lateral.condition.NAV);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_TO_RWY_TRK(const ap_sm_output *BusAssignment)
{
  real_T R;
  real_T r;
  real_T x;
  x = (BusAssignment->data.Psi_magnetic_deg - (BusAssignment->data.nav_loc_deg + 360.0)) + 360.0;
  if (x == 0.0) {
    r = 0.0;
  } else {
    r = std::fmod(x, 360.0);
    if (r == 0.0) {
      r = 0.0;
    } else {
      if (x < 0.0) {
        r += 360.0;
      }
    }
  }

  x = std::abs(-r);
  if (360.0 - x == 0.0) {
    R = 0.0;
  } else {
    R = std::fmod(360.0 - x, 360.0);
    if (R == 0.0) {
      R = 0.0;
    } else {
      if (360.0 - x < 0.0) {
        R += 360.0;
      }
    }
  }

  if (x < std::abs(R)) {
    R = -r;
  }

  return ((BusAssignment->data.H_radio_ft >= 30.0) && (!BusAssignment->lateral.armed.NAV)) ||
    (!BusAssignment->data.nav_valid) || (std::abs(R) > 20.0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_TRK_entry(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->data.Psi_magnetic_track_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GA_TRK_entry(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_GA_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->data.Psi_magnetic_track_deg;
  AutopilotStateMachine_B.out_g.mode_reversion_TRK_FPA = true;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GA_TRK_during(void)
{
  AutopilotStateMachine_B.out_g.mode_reversion_TRK_FPA = false;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_HDG(const ap_sm_output *BusAssignment)
{
  return (BusAssignment->data_computed.time_since_lift_off >= 5.0) && (BusAssignment->input.FD_active ||
    (BusAssignment->output.enabled_AP1 != 0.0) || (BusAssignment->output.enabled_AP2 != 0.0)) &&
    (BusAssignment->input.HDG_pull || (!BusAssignment->lateral.armed.NAV));
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_NAV(const ap_sm_output *BusAssignment)
{
  return (BusAssignment->input.FD_active || (BusAssignment->output.enabled_AP1 != 0.0) ||
          (BusAssignment->output.enabled_AP2 != 0.0)) && BusAssignment->lateral.armed.NAV &&
    BusAssignment->lateral.condition.NAV;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_RWY(const ap_sm_output *BusAssignment)
{
  real_T R;
  real_T r;
  real_T x;
  x = (BusAssignment->data.Psi_magnetic_deg - (BusAssignment->data.nav_loc_deg + 360.0)) + 360.0;
  if (x == 0.0) {
    r = 0.0;
  } else {
    r = std::fmod(x, 360.0);
    if (r == 0.0) {
      r = 0.0;
    } else {
      if (x < 0.0) {
        r += 360.0;
      }
    }
  }

  x = std::abs(-r);
  if (360.0 - x == 0.0) {
    R = 0.0;
  } else {
    R = std::fmod(360.0 - x, 360.0);
    if (R == 0.0) {
      R = 0.0;
    } else {
      if (360.0 - x < 0.0) {
        R += 360.0;
      }
    }
  }

  if (x < std::abs(R)) {
    R = -r;
  }

  return (BusAssignment->input.FD_active || (BusAssignment->output.enabled_AP1 != 0.0) ||
          (BusAssignment->output.enabled_AP2 != 0.0)) && (BusAssignment->data.V2_kn > 100.0) &&
    (BusAssignment->data.flaps_handle_index >= 1.0) && (BusAssignment->data_computed.time_since_touchdown >= 30.0) &&
    ((BusAssignment->data.throttle_lever_1_pos >= 35.0) || (BusAssignment->data.throttle_lever_2_pos >= 35.0)) &&
    BusAssignment->data.nav_valid && ((std::abs(BusAssignment->data.nav_loc_error_deg) <= 0.4) && (std::abs(R) <= 20.0));
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_RWY_TRK(const ap_sm_output *BusAssignment)
{
  return (BusAssignment->input.FD_active || (BusAssignment->output.enabled_AP1 != 0.0) ||
          (BusAssignment->output.enabled_AP2 != 0.0)) && ((!BusAssignment->lateral.armed.NAV) ||
    (BusAssignment->lateral.armed.NAV && (!BusAssignment->lateral.condition.NAV))) && (BusAssignment->data.H_radio_ft >=
    30.0) && (BusAssignment->data.H_radio_ft < 100.0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY;
  AutopilotStateMachine_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS_GA_during(void)
{
  real_T y;
  boolean_T allEnginesOperative;
  AutopilotStateMachine_B.out.FD_connect = false;
  allEnginesOperative = (AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
    AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2);
  if (!AutopilotStateMachine_DWork.wereAllEnginesOperative_not_empty_a) {
    AutopilotStateMachine_DWork.wereAllEnginesOperative_d = allEnginesOperative;
    AutopilotStateMachine_DWork.wereAllEnginesOperative_not_empty_a = true;
  }

  if (AutopilotStateMachine_DWork.wereAllEnginesOperative_d && (!allEnginesOperative)) {
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn) {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn;
    } else {
      y = AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn;
    }

    if (AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 15.0 < y) {
      AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 15.0;
    } else {
      AutopilotStateMachine_B.out.V_c_kn = y;
    }
  }

  AutopilotStateMachine_DWork.wereAllEnginesOperative_d = allEnginesOperative;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_entry_p(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_NONE;
  AutopilotStateMachine_B.out.law = vertical_law_NONE;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT_entry(void)
{
  AutopilotStateMachine_DWork.local_H_fcu_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_ACQ;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS_entry(void)
{
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  if (AutopilotStateMachine_B.BusAssignment_g.input.TRK_FPA_mode) {
    AutopilotStateMachine_B.out.mode = vertical_mode_FPA;
    AutopilotStateMachine_B.out.law = vertical_law_FPA;
  } else {
    AutopilotStateMachine_B.out.mode = vertical_mode_VS;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
  }

  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm;
  AutopilotStateMachine_B.out.FPA_c_deg = AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES_entry(void)
{
  real_T tmp;
  AutopilotStateMachine_B.out.mode = vertical_mode_DES;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (AutopilotStateMachine_B.BusAssignment_g.data_computed.H_constraint_valid) {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
  } else {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  }

  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft - tmp) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = -1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB_entry(void)
{
  real_T tmp;
  AutopilotStateMachine_B.out.mode = vertical_mode_CLB;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (AutopilotStateMachine_B.BusAssignment_g.data_computed.H_constraint_valid) {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
  } else {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  }

  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft - tmp) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_OP_CLB;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }

  AutopilotStateMachine_B.out.EXPED_mode_active = AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_DES_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_OP_DES;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = -1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }

  AutopilotStateMachine_B.out.EXPED_mode_active = AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_GS_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_X_TO_SRS_GA(void)
{
  return AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS_GA;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_during(void)
{
  if (AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.FD_disconnect &&
      (!AutopilotStateMachine_B.BusAssignment_g.input.FD_active)) {
    AutopilotStateMachine_B.out.FD_disconnect = false;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS_GA_entry(void)
{
  real_T y;
  AutopilotStateMachine_DWork.local_H_GA_init_ft = AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
  AutopilotStateMachine_B.out.mode = vertical_mode_SRS_GA;
  AutopilotStateMachine_B.out.law = vertical_law_SRS;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
  if (AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
      AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2) {
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn) {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn;
    } else {
      y = AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn;
    }

    if (AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 25.0 < y) {
      AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 25.0;
    } else {
      AutopilotStateMachine_B.out.V_c_kn = y;
    }
  } else {
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn) {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn;
    } else {
      y = AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn;
    }

    if (AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 15.0 < y) {
      AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn + 15.0;
    } else {
      AutopilotStateMachine_B.out.V_c_kn = y;
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS_entry(void)
{
  real_T y;
  AutopilotStateMachine_B.out.mode = vertical_mode_SRS;
  AutopilotStateMachine_B.out.law = vertical_law_SRS;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
  if (AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
      AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2) {
    AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.V2_kn + 10.0;
  } else {
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > AutopilotStateMachine_B.BusAssignment_g.data.V2_kn) {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn;
    } else {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V2_kn;
    }

    if (AutopilotStateMachine_B.BusAssignment_g.data.V2_kn + 15.0 < y) {
      AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.V2_kn + 15.0;
    } else {
      AutopilotStateMachine_B.out.V_c_kn = y;
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS_during(void)
{
  real_T absx;
  real_T b_absx;
  real_T b_x_tmp;
  real_T c_absx;
  int8_T n;
  if (AutopilotStateMachine_B.BusAssignment_g.input.TRK_FPA_mode) {
    AutopilotStateMachine_B.out.mode = vertical_mode_FPA;
    AutopilotStateMachine_B.out.law = vertical_law_FPA;
  } else {
    AutopilotStateMachine_B.out.mode = vertical_mode_VS;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
  }

  if (!AutopilotStateMachine_DWork.out_H_dot_c_fpm_not_empty) {
    AutopilotStateMachine_DWork.out_H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm;
    AutopilotStateMachine_DWork.out_H_dot_c_fpm_not_empty = true;
  }

  if (!AutopilotStateMachine_DWork.out_FPA_c_deg_not_empty) {
    AutopilotStateMachine_DWork.out_FPA_c_deg = AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg;
    AutopilotStateMachine_DWork.out_FPA_c_deg_not_empty = true;
  }

  if (AutopilotStateMachine_B.out.V_c_kn == AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn) {
    b_x_tmp = AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn - 5.0;
  } else {
    b_x_tmp = AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn;
  }

  if ((AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > b_x_tmp) &&
      (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn < AutopilotStateMachine_B.BusAssignment_g.data.VMAX_kn)) {
    AutopilotStateMachine_B.out.speed_protection_mode = false;
    AutopilotStateMachine_DWork.out_H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm;
    AutopilotStateMachine_DWork.out_FPA_c_deg = AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg;
  } else {
    b_x_tmp = rt_remd(AutopilotStateMachine_B.BusAssignment_g.data.alpha_deg, 360.0);
    c_absx = b_x_tmp;
    b_absx = std::abs(b_x_tmp);
    absx = b_absx;
    if (b_absx > 180.0) {
      if (b_x_tmp > 0.0) {
        c_absx = b_x_tmp - 360.0;
      } else {
        c_absx = b_x_tmp + 360.0;
      }

      absx = std::abs(c_absx);
    }

    if (absx <= 45.0) {
      c_absx *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (c_absx > 0.0) {
        c_absx = (c_absx - 90.0) * 0.017453292519943295;
        n = 1;
      } else {
        c_absx = (c_absx + 90.0) * 0.017453292519943295;
        n = -1;
      }
    } else if (c_absx > 0.0) {
      c_absx = (c_absx - 180.0) * 0.017453292519943295;
      n = 2;
    } else {
      c_absx = (c_absx + 180.0) * 0.017453292519943295;
      n = -2;
    }

    switch (n) {
     case 0:
      c_absx = std::cos(c_absx);
      break;

     case 1:
      c_absx = -std::sin(c_absx);
      break;

     case -1:
      c_absx = std::sin(c_absx);
      break;

     default:
      c_absx = -std::cos(c_absx);
      break;
    }

    if (b_absx > 180.0) {
      if (b_x_tmp > 0.0) {
        b_x_tmp -= 360.0;
      } else {
        b_x_tmp += 360.0;
      }

      b_absx = std::abs(b_x_tmp);
    }

    if (b_absx <= 45.0) {
      b_x_tmp *= 0.017453292519943295;
      n = 0;
    } else if (b_absx <= 135.0) {
      if (b_x_tmp > 0.0) {
        b_x_tmp = (b_x_tmp - 90.0) * 0.017453292519943295;
        n = 1;
      } else {
        b_x_tmp = (b_x_tmp + 90.0) * 0.017453292519943295;
        n = -1;
      }
    } else if (b_x_tmp > 0.0) {
      b_x_tmp = (b_x_tmp - 180.0) * 0.017453292519943295;
      n = 2;
    } else {
      b_x_tmp = (b_x_tmp + 180.0) * 0.017453292519943295;
      n = -2;
    }

    switch (n) {
     case 0:
      b_x_tmp = std::sin(b_x_tmp);
      break;

     case 1:
      b_x_tmp = std::cos(b_x_tmp);
      break;

     case -1:
      b_x_tmp = -std::cos(b_x_tmp);
      break;

     default:
      b_x_tmp = -std::sin(b_x_tmp);
      break;
    }

    b_x_tmp = AutopilotStateMachine_B.BusAssignment_g.time.dt / 6.0 *
      ((AutopilotStateMachine_B.BusAssignment_g.data.ax_m_s2 * c_absx +
        AutopilotStateMachine_B.BusAssignment_g.data.az_m_s2 * b_x_tmp) / 9.81 * 57.295779513082323);
    b_absx = rt_remd(b_x_tmp, 360.0);
    c_absx = std::abs(b_absx);
    if (c_absx > 180.0) {
      if (b_absx > 0.0) {
        b_absx -= 360.0;
      } else {
        b_absx += 360.0;
      }

      c_absx = std::abs(b_absx);
    }

    if (c_absx <= 45.0) {
      b_absx *= 0.017453292519943295;
      n = 0;
    } else if (c_absx <= 135.0) {
      if (b_absx > 0.0) {
        b_absx = (b_absx - 90.0) * 0.017453292519943295;
        n = 1;
      } else {
        b_absx = (b_absx + 90.0) * 0.017453292519943295;
        n = -1;
      }
    } else if (b_absx > 0.0) {
      b_absx = (b_absx - 180.0) * 0.017453292519943295;
      n = 2;
    } else {
      b_absx = (b_absx + 180.0) * 0.017453292519943295;
      n = -2;
    }

    b_absx = std::tan(b_absx);
    if ((n == 1) || (n == -1)) {
      b_absx = -(1.0 / b_absx);
    }

    AutopilotStateMachine_DWork.out_H_dot_c_fpm += AutopilotStateMachine_B.BusAssignment_g.data.V_gnd_kn *
      0.51444444444444448 * b_absx * 3.2808398950131235 * 60.0;
    AutopilotStateMachine_DWork.out_FPA_c_deg += b_x_tmp;
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn < AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn)
    {
      if (AutopilotStateMachine_DWork.out_H_dot_c_fpm >= AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm) {
        AutopilotStateMachine_DWork.out_H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm;
      }

      if (AutopilotStateMachine_DWork.out_FPA_c_deg >= AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg) {
        AutopilotStateMachine_DWork.out_FPA_c_deg = AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg;
      }
    } else {
      if (AutopilotStateMachine_DWork.out_H_dot_c_fpm <= AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm) {
        AutopilotStateMachine_DWork.out_H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm;
      }

      if (AutopilotStateMachine_DWork.out_FPA_c_deg <= AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg) {
        AutopilotStateMachine_DWork.out_FPA_c_deg = AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg;
      }
    }

    AutopilotStateMachine_B.out.speed_protection_mode = ((AutopilotStateMachine_DWork.out_H_dot_c_fpm !=
      AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm) || (AutopilotStateMachine_DWork.out_FPA_c_deg !=
      AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg));
  }

  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.mode_reversion = false;
  AutopilotStateMachine_B.out.H_dot_c_fpm = AutopilotStateMachine_DWork.out_H_dot_c_fpm;
  AutopilotStateMachine_B.out.FPA_c_deg = AutopilotStateMachine_DWork.out_FPA_c_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  AutopilotStateMachine_B.out.ALT_cruise_mode_active = (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
    AutopilotStateMachine_B.BusAssignment_g.data.cruise_altitude) < 60.0);
  AutopilotStateMachine_B.out.ALT_soft_mode_active = AutopilotStateMachine_B.BusAssignment_g.data_computed.ALT_soft_mode;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS(void)
{
  real_T tmp;
  boolean_T tmp_0;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft - AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
    tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
              AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
    if (tmp_0 && (tmp < -50.0)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (tmp_0 && (tmp > 50.0)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else {
      tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                     AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
      if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
          ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                 ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                  AutopilotStateMachine_B.BusAssignment_g.input.ALT_push) &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT;
        AutopilotStateMachine_ALT_entry();
      } else {
        AutopilotStateMachine_VS_during();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_during(void)
{
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (AutopilotStateMachine_DWork.verticalSpeedCancelMode && (std::abs
       (AutopilotStateMachine_B.BusAssignment_g.data.H_dot_ft_min) <= 50.0)) {
    AutopilotStateMachine_DWork.verticalSpeedCancelMode = false;
    AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  }

  if (((AutopilotStateMachine_B.BusAssignment_g.input.AP_1_push &&
        (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 != 0.0)) ||
       (AutopilotStateMachine_B.BusAssignment_g.input.AP_2_push &&
        (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 != 0.0))) && (std::abs
       (AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft - AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) >
       250.0)) {
    AutopilotStateMachine_DWork.verticalSpeedCancelMode = true;
  } else {
    AutopilotStateMachine_DWork.verticalSpeedCancelMode = (AutopilotStateMachine_B.BusAssignment_g.input.Slew_trigger ||
      AutopilotStateMachine_DWork.verticalSpeedCancelMode);
  }

  if (AutopilotStateMachine_DWork.verticalSpeedCancelMode) {
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
    AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
  }

  AutopilotStateMachine_B.out.ALT_cruise_mode_active = (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
    AutopilotStateMachine_B.BusAssignment_g.data.cruise_altitude) < 60.0);
  AutopilotStateMachine_B.out.ALT_soft_mode_active = AutopilotStateMachine_B.BusAssignment_g.data_computed.ALT_soft_mode;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_exit(void)
{
  AutopilotStateMachine_B.out.ALT_cruise_mode_active = false;
  AutopilotStateMachine_B.out.ALT_soft_mode_active = false;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CST;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT(void)
{
  real_T tmp;
  boolean_T tmp_0;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_ALT_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft - AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
    tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
              AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
    if (tmp_0 && (tmp < -50.0)) {
      AutopilotStateMachine_ALT_exit();
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (tmp_0 && (tmp > 50.0)) {
      AutopilotStateMachine_ALT_exit();
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else {
      tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                     AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
      if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
          ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_ALT_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                 ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_ALT_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_ALT_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
        AutopilotStateMachine_ALT_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                 (AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft != 0.0) && (tmp < 50.0)) {
        AutopilotStateMachine_ALT_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST;
        AutopilotStateMachine_ALT_CST_entry();
      } else {
        AutopilotStateMachine_ALT_during();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT(void)
{
  real_T tmp;
  boolean_T tmp_0;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft - AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
    tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
              AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
    if (tmp_0 && (tmp < -50.0)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (tmp_0 && (tmp > 50.0)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else {
      tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                     AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
      if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
          ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                 ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT;
        AutopilotStateMachine_ALT_entry();
      } else if ((std::abs(AutopilotStateMachine_DWork.local_H_fcu_ft -
                           AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft) > 250.0) ||
                 AutopilotStateMachine_B.BusAssignment_g.input.Slew_trigger) {
        AutopilotStateMachine_B.out.mode_reversion = true;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else {
        AutopilotStateMachine_ALT_CPT_during();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST(void)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T tmp_0;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    if ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) ||
        (AutopilotStateMachine_DWork.local_H_constraint_ft !=
         AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft)) {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      } else {
        guard4 = true;
      }
    } else {
      guard4 = true;
    }

    if (guard4) {
      if ((!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) &&
          (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES)) {
        tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
          AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
        if (tmp > 50.0) {
          guard1 = true;
        } else if (tmp < -50.0) {
          guard2 = true;
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    }

    if (guard3) {
      if (((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) ||
           (AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft ==
            AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft)) &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT;
        AutopilotStateMachine_ALT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else {
        tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
          AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
        tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                  AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
        if (tmp_0 && (tmp < -50.0)) {
          guard2 = true;
        } else {
          if (tmp_0 && (tmp > 50.0)) {
            guard1 = true;
          }
        }
      }
    }

    if (guard2) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    }

    if (guard1) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT(void)
{
  real_T tmp;
  real_T tmp_0;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  boolean_T tmp_1;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST;
    AutopilotStateMachine_ALT_CST_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp_0 = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
      AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
    tmp_1 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
              AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
    guard1 = false;
    guard2 = false;
    guard3 = false;
    guard4 = false;
    guard5 = false;
    guard6 = false;
    if (tmp_1 && (tmp_0 < -50.0)) {
      guard1 = true;
    } else if (tmp_1 && (tmp_0 > 50.0)) {
      guard2 = true;
    } else {
      tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                     AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
      if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
          ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        guard3 = true;
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                 ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
        guard4 = true;
      } else if ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) ||
                 (AutopilotStateMachine_DWork.local_H_constraint_ft !=
                  AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft)) {
        if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) {
          guard3 = true;
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES) {
          guard4 = true;
        } else {
          guard6 = true;
        }
      } else {
        guard6 = true;
      }
    }

    if (guard6) {
      if ((!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) &&
          (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES)) {
        if (tmp_0 > 50.0) {
          guard2 = true;
        } else if (tmp_0 < -50.0) {
          guard1 = true;
        } else {
          guard5 = true;
        }
      } else {
        guard5 = true;
      }
    }

    if (guard5) {
      if ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) ||
          (AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft ==
           AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else {
        if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
            AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        }
      }
    }

    if (guard4) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
      AutopilotStateMachine_DES_entry();
    }

    if (guard3) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    }

    if (guard2) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    }

    if (guard1) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB_during(void)
{
  real_T targetAltitude;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (AutopilotStateMachine_B.BusAssignment_g.data_computed.H_constraint_valid) {
    targetAltitude = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
    AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
  } else {
    targetAltitude = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
    AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  }

  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft - targetAltitude) > 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CST_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_ACQ;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES_during(void)
{
  real_T targetAltitude;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (AutopilotStateMachine_B.BusAssignment_g.data_computed.H_constraint_valid) {
    targetAltitude = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
    AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
  } else {
    targetAltitude = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
    AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  }

  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft - targetAltitude) > 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES(void)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T tmp_0;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT) {
    AutopilotStateMachine_DWork.local_H_constraint_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
    AutopilotStateMachine_ALT_CST_CPT_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
    AutopilotStateMachine_ALT_CPT_entry();
  } else {
    guard1 = false;
    if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push || AutopilotStateMachine_B.BusAssignment_g.input.VS_pull)
    {
      guard1 = true;
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
               AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
      AutopilotStateMachine_GS_CPT_entry();
    } else {
      tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
        AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
      if ((!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES) ||
          ((AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft >
            AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) && (std::abs(tmp) > 50.0))) {
        AutopilotStateMachine_B.out.mode_reversion = true;
        guard1 = true;
      } else {
        tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                  AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
        if (tmp_0 && (tmp < -50.0)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (tmp_0 && (tmp > 50.0)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
          AutopilotStateMachine_OP_CLB_entry();
        } else {
          AutopilotStateMachine_DES_during();
        }
      }
    }

    if (guard1) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
      AutopilotStateMachine_VS_entry();
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ROLL_OUT_entry_o(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ROLL_OUT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
  AutopilotStateMachine_B.out.law = vertical_law_FLARE;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_GS_TO_X(void)
{
  return AutopilotStateMachine_B.BusAssignment_g.input.LOC_push ||
    AutopilotStateMachine_B.BusAssignment_g.input.APPR_push;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS_TRACK_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_GS_TRACK;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LAND_entry_i(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_LAND;
  AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_FLARE_entry_g(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_FLARE;
  if ((AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 != 0.0) ||
      (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 != 0.0)) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_SPEED;
  }

  AutopilotStateMachine_B.out.law = vertical_law_FLARE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS(void)
{
  boolean_T guard1 = false;
  guard1 = false;
  switch (AutopilotStateMachine_DWork.is_GS) {
   case AutopilotStateMachine_IN_FLARE:
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT) {
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_ROLL_OUT;
      AutopilotStateMachine_ROLL_OUT_entry_o();
    }
    break;

   case AutopilotStateMachine_IN_GS_CPT:
    if (AutopilotStateMachine_GS_TO_X()) {
      if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
        AutopilotStateMachine_OFF_entry_p();
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    break;

   case AutopilotStateMachine_IN_GS_TRACK:
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.LAND) {
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_LAND_k;
      AutopilotStateMachine_LAND_entry_i();
    } else {
      if (AutopilotStateMachine_GS_TO_X()) {
        if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
            AutopilotStateMachine_OFF_entry_p();
          }
        }
      }
    }
    break;

   case AutopilotStateMachine_IN_LAND_k:
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.FLARE) {
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_FLARE;
      AutopilotStateMachine_FLARE_entry_g();
    }
    break;

   default:
    if (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT) {
      if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else {
        if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
          AutopilotStateMachine_OFF_entry_p();
        }
      }
    }
    break;
  }

  if (guard1) {
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_TRACK) {
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_TRACK;
      AutopilotStateMachine_GS_TRACK_entry();
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft) > 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
  }

  if (AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) {
    AutopilotStateMachine_B.out.EXPED_mode_active =
      !AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.EXPED_mode_active;
  } else {
    if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull) {
      AutopilotStateMachine_B.out.EXPED_mode_active = false;
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_exit(void)
{
  AutopilotStateMachine_B.out.EXPED_mode_active = false;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_DES_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft;
  AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft) > 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_requested_mode_THRUST_IDLE;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
  }

  if (AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) {
    AutopilotStateMachine_B.out.EXPED_mode_active =
      !AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.EXPED_mode_active;
  } else {
    if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull) {
      AutopilotStateMachine_B.out.EXPED_mode_active = false;
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS_during(void)
{
  real_T y;
  boolean_T allEnginesOperative;
  allEnginesOperative = (AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
    AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2);
  if (!AutopilotStateMachine_DWork.wereAllEnginesOperative_not_empty) {
    AutopilotStateMachine_DWork.wereAllEnginesOperative = allEnginesOperative;
    AutopilotStateMachine_DWork.wereAllEnginesOperative_not_empty = true;
  }

  if (AutopilotStateMachine_DWork.wereAllEnginesOperative && (!allEnginesOperative)) {
    if (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn > AutopilotStateMachine_B.BusAssignment_g.data.V2_kn) {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn;
    } else {
      y = AutopilotStateMachine_B.BusAssignment_g.data.V2_kn;
    }

    if (AutopilotStateMachine_B.BusAssignment_g.data.V2_kn + 15.0 < y) {
      AutopilotStateMachine_B.out.V_c_kn = AutopilotStateMachine_B.BusAssignment_g.data.V2_kn + 15.0;
    } else {
      AutopilotStateMachine_B.out.V_c_kn = y;
    }
  }

  AutopilotStateMachine_DWork.wereAllEnginesOperative = allEnginesOperative;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS(void)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T tmp_0;
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
      AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2) {
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
        AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) {
      guard1 = true;
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES &&
               AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES) {
      guard2 = true;
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
               AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
      AutopilotStateMachine_ALT_CPT_entry();
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    if ((AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) &&
        (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
      AutopilotStateMachine_OFF_entry_p();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
               AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
      AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
      AutopilotStateMachine_GS_CPT_entry();
    } else {
      tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
        AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
      tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
               AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
      if (tmp_0 && (tmp < -50.0)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
        AutopilotStateMachine_OP_DES_entry();
      } else if (tmp_0 && (tmp > 50.0)) {
        guard3 = true;
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                       AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
            ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          guard1 = true;
        } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                   ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          guard2 = true;
        } else if ((AutopilotStateMachine_B.BusAssignment_g.data_computed.V_fcu_in_selection &&
                    (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0)) ||
                   ((!AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB) &&
                    (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 2.0) &&
                    AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
                    AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2)) {
          guard3 = true;
        } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                   AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else {
          AutopilotStateMachine_SRS_during();
        }
      }
    }
  }

  if (guard3) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
    AutopilotStateMachine_OP_CLB_entry();
  }

  if (guard2) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
    AutopilotStateMachine_DES_entry();
  }

  if (guard1) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
    AutopilotStateMachine_CLB_entry();
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_exit_internal_ON(void)
{
  switch (AutopilotStateMachine_DWork.is_ON) {
   case AutopilotStateMachine_IN_ALT:
    AutopilotStateMachine_ALT_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    break;

   case AutopilotStateMachine_IN_OP_CLB:
    AutopilotStateMachine_OP_CLB_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    break;

   case AutopilotStateMachine_IN_OP_DES:
    AutopilotStateMachine_OP_CLB_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    break;

   default:
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    break;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ON(void)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  boolean_T tmp_0;
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  guard5 = false;
  guard6 = false;
  if (((!AutopilotStateMachine_B.BusAssignment_g.input.FD_active) &&
       (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 == 0.0) &&
       (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 == 0.0)) ||
      (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 7.0) ||
      ((AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 0.0) &&
       (AutopilotStateMachine_B.BusAssignment_g.data.throttle_lever_1_pos < 35.0) &&
       (AutopilotStateMachine_B.BusAssignment_g.data.throttle_lever_2_pos < 35.0))) {
    guard6 = true;
  } else if (AutopilotStateMachine_X_TO_SRS_GA()) {
    AutopilotStateMachine_exit_internal_ON();
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_SRS_GA;
    AutopilotStateMachine_SRS_GA_entry();
  } else if ((AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 == 0.0) &&
             (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 == 0.0) &&
             (((AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.mode == vertical_mode_OP_CLB) &&
               (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn >=
                AutopilotStateMachine_B.BusAssignment_g.data.VMAX_kn + 4.0)) ||
              ((AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.mode == vertical_mode_OP_DES) &&
               (AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn <=
                AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn - 2.0)))) {
    AutopilotStateMachine_B.out.FD_disconnect = true;
    guard6 = true;
  } else {
    switch (AutopilotStateMachine_DWork.is_ON) {
     case AutopilotStateMachine_IN_ALT:
      AutopilotStateMachine_ALT();
      break;

     case AutopilotStateMachine_IN_ALT_CPT:
      AutopilotStateMachine_ALT_CPT();
      break;

     case AutopilotStateMachine_IN_ALT_CST:
      AutopilotStateMachine_ALT_CST();
      break;

     case AutopilotStateMachine_IN_ALT_CST_CPT:
      AutopilotStateMachine_ALT_CST_CPT();
      break;

     case AutopilotStateMachine_IN_CLB:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT) {
        AutopilotStateMachine_DWork.local_H_constraint_ft =
          AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
        AutopilotStateMachine_ALT_CST_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
        guard1 = true;
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
          AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
        tmp_0 = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                  AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
        if (tmp_0 && (tmp < -50.0)) {
          guard2 = true;
        } else if (tmp_0 && (tmp > 50.0)) {
          guard3 = true;
        } else if ((AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft <
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) && (std::abs
                    (AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
                     AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) > 50.0)) {
          AutopilotStateMachine_B.out.mode_reversion = true;
          guard1 = true;
        } else if ((!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) && (tmp > 50.0)) {
          if (tmp > 50.0) {
            guard3 = true;
          } else if (tmp < -50.0) {
            guard2 = true;
          } else {
            AutopilotStateMachine_CLB_during();
          }
        } else {
          AutopilotStateMachine_CLB_during();
        }
      }
      break;

     case AutopilotStateMachine_IN_DES:
      AutopilotStateMachine_DES();
      break;

     case AutopilotStateMachine_IN_GS:
      AutopilotStateMachine_GS();
      break;

     case AutopilotStateMachine_IN_OP_CLB:
      if ((AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft <
           AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) && (std::abs
           (AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
            AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) > 50.0)) {
        AutopilotStateMachine_B.out.mode_reversion = true;
        guard4 = true;
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_OP_CLB_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
        guard4 = true;
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_OP_CLB_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                       AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
            ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          AutopilotStateMachine_OP_CLB_exit();
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
          AutopilotStateMachine_CLB_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                   ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          AutopilotStateMachine_OP_CLB_exit();
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else {
          AutopilotStateMachine_OP_CLB_during();
        }
      }
      break;

     case AutopilotStateMachine_IN_OP_DES:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_OP_CLB_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
        guard5 = true;
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_OP_CLB_exit();
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else if ((AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft >
                  AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) && (std::abs
                  (AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft) > 50.0)) {
        AutopilotStateMachine_B.out.mode_reversion = true;
        guard5 = true;
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
                       AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
            AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
            ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          AutopilotStateMachine_OP_CLB_exit();
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
          AutopilotStateMachine_CLB_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                   AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                   ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (tmp > 50.0))) {
          AutopilotStateMachine_OP_CLB_exit();
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else {
          AutopilotStateMachine_OP_DES_during();
        }
      }
      break;

     case AutopilotStateMachine_IN_SRS:
      AutopilotStateMachine_SRS();
      break;

     default:
      AutopilotStateMachine_VS();
      break;
    }
  }

  if (guard6) {
    AutopilotStateMachine_exit_internal_ON();
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
    AutopilotStateMachine_OFF_entry_p();
  }

  if (guard5) {
    AutopilotStateMachine_OP_CLB_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
    AutopilotStateMachine_VS_entry();
  }

  if (guard4) {
    AutopilotStateMachine_OP_CLB_exit();
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
    AutopilotStateMachine_VS_entry();
  }

  if (guard3) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
    AutopilotStateMachine_OP_CLB_entry();
  }

  if (guard2) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
    AutopilotStateMachine_OP_DES_entry();
  }

  if (guard1) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
    AutopilotStateMachine_VS_entry();
  }
}

void AutopilotStateMachineModelClass::step()
{
  uint64m_T tmp;
  uint64m_T tmp_0;
  uint64m_T tmp_1;
  uint64m_T tmp_2;
  uint64m_T tmp_3;
  uint64m_T tmp_4;
  uint64m_T tmp_5;
  real_T result_tmp[9];
  real_T result[3];
  real_T result_0[3];
  real_T rtb_DataTypeConversion2_f;
  real_T rtb_DataTypeConversion3_g;
  real_T rtb_DataTypeConversion_h;
  real_T rtb_Gain2;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  real_T rtb_Switch_d;
  real_T rtb_y_o;
  real_T rtb_y_p;
  real_T u;
  int32_T rtb_Divide_0;
  int32_T rtb_on_ground;
  boolean_T conditionSoftAlt;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T rtb_AND;
  boolean_T rtb_BusAssignment1_input_APPR_push;
  boolean_T rtb_BusAssignment1_input_LOC_push;
  boolean_T rtb_FixPtRelationalOperator;
  boolean_T rtb_Y_j;
  boolean_T rtb_cFLARE;
  boolean_T rtb_cGA;
  boolean_T rtb_cLAND;
  boolean_T sCLB_tmp;
  boolean_T speedTargetChanged;
  boolean_T state_h_tmp;
  boolean_T throttleCondition;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_p = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.AP_1_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_p));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.AP_2_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_b));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = (static_cast<int32_T>
    (AutopilotStateMachine_U.in.input.AP_DISCONNECT_push) > static_cast<int32_T>
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_d));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_e));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_g = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_pull) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_g));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_f));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_pull) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_i));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_a = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_pull) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_a));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.LOC_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.APPR_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_h));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.EXPED_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_o));
  rtb_GainTheta = AutopilotStateMachine_P.GainTheta_Gain * AutopilotStateMachine_U.in.data.Theta_deg;
  rtb_GainTheta1 = AutopilotStateMachine_P.GainTheta1_Gain * AutopilotStateMachine_U.in.data.Phi_deg;
  rtb_Saturation = 0.017453292519943295 * rtb_GainTheta;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta1;
  rtb_y_o = std::tan(rtb_Saturation);
  rtb_Gain2 = std::sin(rtb_Saturation1);
  rtb_y_p = std::cos(rtb_Saturation1);
  result_tmp[0] = 1.0;
  result_tmp[3] = rtb_Gain2 * rtb_y_o;
  result_tmp[6] = rtb_y_p * rtb_y_o;
  result_tmp[1] = 0.0;
  result_tmp[4] = rtb_y_p;
  result_tmp[7] = -rtb_Gain2;
  result_tmp[2] = 0.0;
  rtb_DataTypeConversion3_g = 1.0 / std::cos(rtb_Saturation);
  result_tmp[5] = rtb_DataTypeConversion3_g * rtb_Gain2;
  result_tmp[8] = rtb_DataTypeConversion3_g * rtb_y_p;
  rtb_Gain2 = AutopilotStateMachine_P.Gain_Gain_k * AutopilotStateMachine_U.in.data.p_rad_s *
    AutopilotStateMachine_P.Gainpk_Gain;
  rtb_y_p = AutopilotStateMachine_P.Gain_Gain * AutopilotStateMachine_U.in.data.q_rad_s *
    AutopilotStateMachine_P.Gainqk_Gain;
  rtb_y_o = AutopilotStateMachine_P.Gain_Gain_a * AutopilotStateMachine_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = result_tmp[rtb_on_ground + 6] * rtb_y_o + (result_tmp[rtb_on_ground + 3] * rtb_y_p +
      result_tmp[rtb_on_ground] * rtb_Gain2);
  }

  rtb_y_o = std::cos(rtb_Saturation);
  rtb_Gain2 = std::sin(rtb_Saturation);
  rtb_y_p = std::sin(rtb_Saturation1);
  rtb_Saturation = std::cos(rtb_Saturation1);
  result_tmp[0] = rtb_y_o;
  result_tmp[3] = 0.0;
  result_tmp[6] = -rtb_Gain2;
  result_tmp[1] = rtb_y_p * rtb_Gain2;
  result_tmp[4] = rtb_Saturation;
  result_tmp[7] = rtb_y_o * rtb_y_p;
  result_tmp[2] = rtb_Saturation * rtb_Gain2;
  result_tmp[5] = 0.0 - rtb_y_p;
  result_tmp[8] = rtb_Saturation * rtb_y_o;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result_0[rtb_on_ground] = result_tmp[rtb_on_ground + 6] * AutopilotStateMachine_U.in.data.bz_m_s2 +
      (result_tmp[rtb_on_ground + 3] * AutopilotStateMachine_U.in.data.by_m_s2 + result_tmp[rtb_on_ground] *
       AutopilotStateMachine_U.in.data.bx_m_s2);
  }

  rtb_Saturation = AutopilotStateMachine_P.Gain_Gain_af * AutopilotStateMachine_U.in.data.gear_strut_compression_1 -
    AutopilotStateMachine_P.Constant1_Value;
  if (rtb_Saturation > AutopilotStateMachine_P.Saturation_UpperSat) {
    rtb_Saturation = AutopilotStateMachine_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < AutopilotStateMachine_P.Saturation_LowerSat) {
      rtb_Saturation = AutopilotStateMachine_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = AutopilotStateMachine_P.Gain1_Gain * AutopilotStateMachine_U.in.data.gear_strut_compression_2 -
    AutopilotStateMachine_P.Constant1_Value;
  if (rtb_Saturation1 > AutopilotStateMachine_P.Saturation1_UpperSat) {
    rtb_Saturation1 = AutopilotStateMachine_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < AutopilotStateMachine_P.Saturation1_LowerSat) {
      rtb_Saturation1 = AutopilotStateMachine_P.Saturation1_LowerSat;
    }
  }

  if (AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_OnGround;
    rtb_on_ground = 1;
  } else if (AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine == 1) {
    if ((rtb_Saturation > 0.05) || (rtb_Saturation1 > 0.05)) {
      AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_BusAssignment1_input_LOC_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn;
  rtb_BusAssignment1_input_APPR_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_h;
  AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  if (!AutopilotStateMachine_DWork.eventTime_not_empty_p) {
    AutopilotStateMachine_DWork.eventTime_p = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty_p = true;
  }

  if ((rtb_on_ground == 0) || (AutopilotStateMachine_DWork.eventTime_p == 0.0)) {
    AutopilotStateMachine_DWork.eventTime_p = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_y_p = AutopilotStateMachine_U.in.time.simulation_time - AutopilotStateMachine_DWork.eventTime_p;
  if (!AutopilotStateMachine_DWork.eventTime_not_empty_h) {
    AutopilotStateMachine_DWork.eventTime_i = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty_h = true;
  }

  if ((rtb_on_ground != 0) || (AutopilotStateMachine_DWork.eventTime_i == 0.0)) {
    AutopilotStateMachine_DWork.eventTime_i = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_y_o = AutopilotStateMachine_U.in.time.simulation_time - AutopilotStateMachine_DWork.eventTime_i;
  if (!AutopilotStateMachine_DWork.eventTime_not_empty) {
    AutopilotStateMachine_DWork.eventTime = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty = true;
  }

  if (((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS)) &&
       (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS_GA))) ||
      (AutopilotStateMachine_DWork.eventTime == 0.0)) {
    AutopilotStateMachine_DWork.eventTime = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_Saturation = AutopilotStateMachine_P.Constant_Value_j / AutopilotStateMachine_U.in.time.dt;
  if (rtb_Saturation < 1.0) {
    rtb_DataTypeConversion3_g = AutopilotStateMachine_U.in.input.H_fcu_ft;
  } else {
    if (rtb_Saturation > 100.0) {
      rtb_Divide_0 = 100;
    } else {
      rtb_Divide_0 = static_cast<int32_T>(static_cast<uint32_T>(std::fmod(std::floor(rtb_Saturation), 4.294967296E+9)));
    }

    rtb_DataTypeConversion3_g = AutopilotStateMachine_DWork.Delay_DSTATE_d[100U - rtb_Divide_0];
  }

  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (rtb_DataTypeConversion3_g !=
    AutopilotStateMachine_U.in.input.H_fcu_ft);
  rtb_Y_j = ((AutopilotStateMachine_U.in.input.H_constraint_ft != 0.0) && (((AutopilotStateMachine_U.in.input.H_fcu_ft >
    AutopilotStateMachine_U.in.data.H_ind_ft) && (AutopilotStateMachine_U.in.input.H_constraint_ft >
    AutopilotStateMachine_U.in.data.H_ind_ft) && (AutopilotStateMachine_U.in.input.H_constraint_ft <
    AutopilotStateMachine_U.in.input.H_fcu_ft)) || ((AutopilotStateMachine_U.in.input.H_fcu_ft <
    AutopilotStateMachine_U.in.data.H_ind_ft) && (AutopilotStateMachine_U.in.input.H_constraint_ft <
    AutopilotStateMachine_U.in.data.H_ind_ft) && (AutopilotStateMachine_U.in.input.H_constraint_ft >
    AutopilotStateMachine_U.in.input.H_fcu_ft))));
  rtb_Saturation = AutopilotStateMachine_P.Constant_Value_jq / AutopilotStateMachine_U.in.time.dt;
  if (rtb_Saturation < 1.0) {
    rtb_DataTypeConversion3_g = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  } else {
    if (rtb_Saturation > 100.0) {
      rtb_Divide_0 = 100;
    } else {
      rtb_Divide_0 = static_cast<int32_T>(static_cast<uint32_T>(std::fmod(std::floor(rtb_Saturation), 4.294967296E+9)));
    }

    rtb_DataTypeConversion3_g = AutopilotStateMachine_DWork.Delay_DSTATE_c[100U - rtb_Divide_0];
  }

  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (rtb_DataTypeConversion3_g !=
    AutopilotStateMachine_U.in.input.Psi_fcu_deg);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = (AutopilotStateMachine_U.in.input.Psi_fcu_deg !=
    AutopilotStateMachine_P.CompareToConstant_const);
  rtb_AND = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_h && AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn);
  rtb_DataTypeConversion_h = AutopilotStateMachine_U.in.time.dt * AutopilotStateMachine_P.LagFilter_C1;
  rtb_DataTypeConversion2_f = rtb_DataTypeConversion_h + AutopilotStateMachine_P.Constant_Value_b;
  AutopilotStateMachine_DWork.Delay1_DSTATE_b = 1.0 / rtb_DataTypeConversion2_f *
    (AutopilotStateMachine_P.Constant_Value_b - rtb_DataTypeConversion_h) * AutopilotStateMachine_DWork.Delay1_DSTATE_b
    + (AutopilotStateMachine_U.in.data.nav_gs_error_deg + AutopilotStateMachine_DWork.Delay_DSTATE_l) *
    (rtb_DataTypeConversion_h / rtb_DataTypeConversion2_f);
  rtb_FixPtRelationalOperator = (AutopilotStateMachine_DWork.Delay1_DSTATE_b <
    AutopilotStateMachine_DWork.DelayInput1_DSTATE);
  rtb_DataTypeConversion_h = AutopilotStateMachine_U.in.time.dt * AutopilotStateMachine_P.WashoutFilter_C1;
  rtb_DataTypeConversion3_g = rtb_DataTypeConversion_h + AutopilotStateMachine_P.Constant_Value_l;
  rtb_Switch_d = AutopilotStateMachine_P.Constant_Value_l / rtb_DataTypeConversion3_g;
  rtb_Saturation = AutopilotStateMachine_DWork.Delay_DSTATE_b * rtb_Switch_d;
  rtb_Switch_d *= AutopilotStateMachine_U.in.data.H_ft;
  AutopilotStateMachine_DWork.Delay1_DSTATE_f = 1.0 / rtb_DataTypeConversion3_g *
    (AutopilotStateMachine_P.Constant_Value_l - rtb_DataTypeConversion_h) * AutopilotStateMachine_DWork.Delay1_DSTATE_f
    + (rtb_Switch_d - rtb_Saturation);
  if (AutopilotStateMachine_U.in.data.H_radio_ft > AutopilotStateMachine_P.Saturation_UpperSat_k) {
    rtb_Saturation = AutopilotStateMachine_P.Saturation_UpperSat_k;
  } else if (AutopilotStateMachine_U.in.data.H_radio_ft < AutopilotStateMachine_P.Saturation_LowerSat_b) {
    rtb_Saturation = AutopilotStateMachine_P.Saturation_LowerSat_b;
  } else {
    rtb_Saturation = AutopilotStateMachine_U.in.data.H_radio_ft;
  }

  rtb_Switch_d = AutopilotStateMachine_U.in.time.dt * AutopilotStateMachine_P.LagFilter_C1_n;
  rtb_DataTypeConversion2_f = rtb_Switch_d + AutopilotStateMachine_P.Constant_Value_f;
  AutopilotStateMachine_DWork.Delay1_DSTATE_i = 1.0 / rtb_DataTypeConversion2_f *
    (AutopilotStateMachine_P.Constant_Value_f - rtb_Switch_d) * AutopilotStateMachine_DWork.Delay1_DSTATE_i +
    (rtb_Saturation + AutopilotStateMachine_DWork.Delay_DSTATE_a) * (rtb_Switch_d / rtb_DataTypeConversion2_f);
  rtb_Saturation1 = (AutopilotStateMachine_DWork.Delay1_DSTATE_f + AutopilotStateMachine_DWork.Delay1_DSTATE_i) *
    AutopilotStateMachine_P.DiscreteDerivativeVariableTs2_Gain;
  rtb_Gain2 = (rtb_Saturation1 - AutopilotStateMachine_DWork.Delay_DSTATE_o) / AutopilotStateMachine_U.in.time.dt *
    AutopilotStateMachine_P.Gain2_Gain_d;
  rtb_Switch_d = AutopilotStateMachine_U.in.time.dt * AutopilotStateMachine_P.LagFilter3_C1;
  rtb_DataTypeConversion2_f = rtb_Switch_d + AutopilotStateMachine_P.Constant_Value_jm;
  AutopilotStateMachine_DWork.Delay1_DSTATE_h = 1.0 / rtb_DataTypeConversion2_f *
    (AutopilotStateMachine_P.Constant_Value_jm - rtb_Switch_d) * AutopilotStateMachine_DWork.Delay1_DSTATE_h +
    (rtb_Gain2 + AutopilotStateMachine_DWork.Delay_DSTATE_ov) * (rtb_Switch_d / rtb_DataTypeConversion2_f);
  rtb_DataTypeConversion2_f = AutopilotStateMachine_U.in.time.dt * AutopilotStateMachine_P.WashoutFilter1_C1;
  rtb_DataTypeConversion_h = rtb_DataTypeConversion2_f + AutopilotStateMachine_P.Constant_Value_n;
  rtb_DataTypeConversion3_g = AutopilotStateMachine_P.Constant_Value_n / rtb_DataTypeConversion_h;
  rtb_Switch_d = AutopilotStateMachine_DWork.Delay_DSTATE_c5 * rtb_DataTypeConversion3_g;
  rtb_DataTypeConversion3_g *= AutopilotStateMachine_U.in.data.H_dot_ft_min;
  AutopilotStateMachine_DWork.Delay1_DSTATE_p = 1.0 / rtb_DataTypeConversion_h *
    (AutopilotStateMachine_P.Constant_Value_n - rtb_DataTypeConversion2_f) * AutopilotStateMachine_DWork.Delay1_DSTATE_p
    + (rtb_DataTypeConversion3_g - rtb_Switch_d);
  rtb_DataTypeConversion_h = AutopilotStateMachine_DWork.Delay1_DSTATE_h + AutopilotStateMachine_DWork.Delay1_DSTATE_p;
  rtb_DataTypeConversion3_g = AutopilotStateMachine_P.Constant_Value_m / AutopilotStateMachine_U.in.time.dt;
  if (rtb_DataTypeConversion3_g < 1.0) {
    rtb_Switch_d = AutopilotStateMachine_U.in.input.V_fcu_kn;
  } else {
    if (rtb_DataTypeConversion3_g > 100.0) {
      rtb_Divide_0 = 100;
    } else {
      rtb_Divide_0 = static_cast<int32_T>(static_cast<uint32_T>(std::fmod(std::floor(rtb_DataTypeConversion3_g),
        4.294967296E+9)));
    }

    rtb_Switch_d = AutopilotStateMachine_DWork.Delay_DSTATE_d2[100U - rtb_Divide_0];
  }

  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (rtb_Switch_d != AutopilotStateMachine_U.in.input.V_fcu_kn);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = (AutopilotStateMachine_U.in.input.V_fcu_kn !=
    AutopilotStateMachine_P.CompareToConstant_const_l);
  if (!AutopilotStateMachine_DWork.lastTargetSpeed_not_empty) {
    AutopilotStateMachine_DWork.lastTargetSpeed = AutopilotStateMachine_U.in.input.V_fcu_kn;
    AutopilotStateMachine_DWork.lastTargetSpeed_not_empty = true;
  }

  speedTargetChanged = (std::abs(AutopilotStateMachine_U.in.input.V_fcu_kn - AutopilotStateMachine_DWork.lastTargetSpeed)
                        > 2.0);
  AutopilotStateMachine_DWork.lastTargetSpeed = AutopilotStateMachine_U.in.input.V_fcu_kn;
  rtb_DataTypeConversion3_g = std::abs(AutopilotStateMachine_U.in.input.V_fcu_kn -
    AutopilotStateMachine_U.in.data.V_ias_kn);
  if ((rtb_DataTypeConversion3_g <= 4.0) || (!AutopilotStateMachine_DWork.timeDeltaSpeed4_not_empty)) {
    AutopilotStateMachine_DWork.timeDeltaSpeed4 = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.timeDeltaSpeed4_not_empty = true;
  }

  if ((rtb_DataTypeConversion3_g <= 10.0) || (!AutopilotStateMachine_DWork.timeDeltaSpeed10_not_empty)) {
    AutopilotStateMachine_DWork.timeDeltaSpeed10 = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.timeDeltaSpeed10_not_empty = true;
  }

  conditionSoftAlt = ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_ALT) && ((std::abs
    (AutopilotStateMachine_U.in.data.H_ind_ft - AutopilotStateMachine_U.in.data.cruise_altitude) < 60.0) &&
    (AutopilotStateMachine_U.in.input.MACH_mode && AutopilotStateMachine_U.in.input.ATHR_engaged &&
     (rtb_DataTypeConversion3_g < 4.0))));
  if ((!conditionSoftAlt) || speedTargetChanged || (!AutopilotStateMachine_DWork.timeConditionSoftAlt_not_empty)) {
    AutopilotStateMachine_DWork.timeConditionSoftAlt = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.timeConditionSoftAlt_not_empty = true;
  }

  AutopilotStateMachine_DWork.stateSoftAlt = ((conditionSoftAlt && (AutopilotStateMachine_U.in.time.simulation_time -
    AutopilotStateMachine_DWork.timeConditionSoftAlt >= 120.0)) || AutopilotStateMachine_DWork.stateSoftAlt);
  if (speedTargetChanged || (!AutopilotStateMachine_U.in.input.MACH_mode) ||
      (!AutopilotStateMachine_U.in.input.ATHR_engaged) || (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
       vertical_mode_ALT) || (AutopilotStateMachine_U.in.time.simulation_time -
       AutopilotStateMachine_DWork.timeDeltaSpeed4 > 10.0) || (AutopilotStateMachine_U.in.time.simulation_time -
       AutopilotStateMachine_DWork.timeDeltaSpeed10 > 4.0) || (AutopilotStateMachine_U.in.data.V_ias_kn >
       AutopilotStateMachine_U.in.data.VMAX_kn - 5.0)) {
    AutopilotStateMachine_DWork.stateSoftAlt = false;
    AutopilotStateMachine_DWork.timeConditionSoftAlt = AutopilotStateMachine_U.in.time.simulation_time;
  }

  speedTargetChanged = ((AutopilotStateMachine_U.in.data.H_radio_ft > 100.0) && (rtb_y_o > 5.0));
  conditionSoftAlt = ((AutopilotStateMachine_DWork.Delay_DSTATE.armed.LOC ||
                       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT) ||
                       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK) ||
                       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND) ||
                       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE) ||
                       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_ROLL_OUT)) &&
                      (AutopilotStateMachine_DWork.Delay1_DSTATE.armed.GS ||
                       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_CPT) ||
                       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK) ||
                       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_LAND) ||
                       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FLARE) ||
                       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_ROLL_OUT)));
  if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_p) {
    if (!AutopilotStateMachine_DWork.sAP1) {
      if (speedTargetChanged) {
        AutopilotStateMachine_DWork.sAP1 = true;
        AutopilotStateMachine_DWork.sAP2 = (conditionSoftAlt && AutopilotStateMachine_DWork.sAP2);
      }
    } else {
      AutopilotStateMachine_DWork.sAP1 = false;
      AutopilotStateMachine_DWork.sAP2 = false;
    }
  } else if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_b) {
    if (!AutopilotStateMachine_DWork.sAP2) {
      if (speedTargetChanged) {
        AutopilotStateMachine_DWork.sAP2 = true;
        AutopilotStateMachine_DWork.sAP1 = (conditionSoftAlt && AutopilotStateMachine_DWork.sAP1);
      }
    } else {
      AutopilotStateMachine_DWork.sAP1 = false;
      AutopilotStateMachine_DWork.sAP2 = false;
    }
  } else if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_d) {
    AutopilotStateMachine_DWork.sAP1 = false;
    AutopilotStateMachine_DWork.sAP2 = false;
  } else {
    if ((!conditionSoftAlt) && AutopilotStateMachine_DWork.sLandModeArmedOrActive && AutopilotStateMachine_DWork.sAP1 &&
        AutopilotStateMachine_DWork.sAP2) {
      AutopilotStateMachine_DWork.sAP1 = true;
      AutopilotStateMachine_DWork.sAP2 = false;
    }
  }

  AutopilotStateMachine_DWork.sLandModeArmedOrActive = conditionSoftAlt;
  AutopilotStateMachine_DWork.state_h = ((AutopilotStateMachine_U.in.data.is_flight_plan_available &&
    (!AutopilotStateMachine_DWork.Delay_DSTATE.condition.NAV) && AutopilotStateMachine_DWork.DelayInput1_DSTATE_e &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_NAV) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_CPT) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_TRACK) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LAND) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_FLARE)) || AutopilotStateMachine_DWork.state_h);
  state_h_tmp = !AutopilotStateMachine_DWork.Delay_DSTATE.armed.LOC;
  AutopilotStateMachine_DWork.state_h = ((!AutopilotStateMachine_DWork.DelayInput1_DSTATE_g) &&
    ((AutopilotStateMachine_U.in.data.H_radio_ft >= 30.0) || (!rtb_AND)) && state_h_tmp &&
    (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_NAV)) &&
    (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND)) &&
    (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE)) &&
    AutopilotStateMachine_DWork.state_h);
  rtb_cLAND = (AutopilotStateMachine_DWork.Delay1_DSTATE.armed.GS ||
               (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_CPT) ||
               (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK) ||
               (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_LAND) ||
               (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FLARE));
  rtb_cFLARE = !rtb_cLAND;
  rtb_cGA = ((!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT)) &&
             (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK)) &&
             (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND)) &&
             (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE)));
  AutopilotStateMachine_DWork.state_d = (((AutopilotStateMachine_U.in.data.H_radio_ft > 400.0) &&
    AutopilotStateMachine_U.in.data.nav_valid && (AutopilotStateMachine_U.in.data.throttle_lever_1_pos < 45.0) &&
    (AutopilotStateMachine_U.in.data.throttle_lever_2_pos < 45.0) && (rtb_BusAssignment1_input_LOC_push ||
    rtb_BusAssignment1_input_APPR_push) && rtb_cGA && rtb_cFLARE) || AutopilotStateMachine_DWork.state_d);
  speedTargetChanged = !rtb_BusAssignment1_input_APPR_push;
  conditionSoftAlt = !rtb_BusAssignment1_input_LOC_push;
  AutopilotStateMachine_DWork.state_d = ((conditionSoftAlt || state_h_tmp || rtb_cLAND) && (speedTargetChanged ||
    rtb_cFLARE) && (!AutopilotStateMachine_DWork.Delay_DSTATE.armed.NAV) && rtb_cGA &&
    (AutopilotStateMachine_U.in.data.throttle_lever_1_pos != 45.0) &&
    (AutopilotStateMachine_U.in.data.throttle_lever_2_pos != 45.0) && AutopilotStateMachine_DWork.state_d);
  rtb_DataTypeConversion2_f = (AutopilotStateMachine_U.in.data.Psi_magnetic_deg -
    (AutopilotStateMachine_U.in.data.nav_loc_deg + 360.0)) + 360.0;
  if (rtb_DataTypeConversion2_f == 0.0) {
    rtb_DataTypeConversion3_g = 0.0;
  } else {
    rtb_DataTypeConversion3_g = std::fmod(rtb_DataTypeConversion2_f, 360.0);
    if (rtb_DataTypeConversion3_g == 0.0) {
      rtb_DataTypeConversion3_g = 0.0;
    } else {
      if (rtb_DataTypeConversion2_f < 0.0) {
        rtb_DataTypeConversion3_g += 360.0;
      }
    }
  }

  if (360.0 - rtb_DataTypeConversion3_g == 0.0) {
    rtb_Switch_d = 0.0;
  } else {
    rtb_Switch_d = std::fmod(360.0 - rtb_DataTypeConversion3_g, 360.0);
    if (rtb_Switch_d == 0.0) {
      rtb_Switch_d = 0.0;
    } else {
      if (360.0 - rtb_DataTypeConversion3_g < 0.0) {
        rtb_Switch_d += 360.0;
      }
    }
  }

  if (rtb_DataTypeConversion3_g < rtb_Switch_d) {
    rtb_Switch_d = -rtb_DataTypeConversion3_g;
  }

  if (AutopilotStateMachine_U.in.data.nav_valid && AutopilotStateMachine_U.in.data.nav_loc_valid) {
    rtb_DataTypeConversion3_g = std::abs(rtb_Switch_d);
    if (rtb_DataTypeConversion3_g < 115.0) {
      rtb_DataTypeConversion2_f = std::abs(AutopilotStateMachine_U.in.data.nav_loc_error_deg);
      if (rtb_DataTypeConversion2_f < 3.0) {
        if (rtb_Switch_d < 0.0) {
          rtb_Switch_d = -1.0;
        } else {
          if (rtb_Switch_d > 0.0) {
            rtb_Switch_d = 1.0;
          }
        }

        if (AutopilotStateMachine_U.in.input.Phi_loc_c < 0.0) {
          u = -1.0;
        } else if (AutopilotStateMachine_U.in.input.Phi_loc_c > 0.0) {
          u = 1.0;
        } else {
          u = AutopilotStateMachine_U.in.input.Phi_loc_c;
        }

        if ((rtb_Switch_d == u) && (std::abs(AutopilotStateMachine_U.in.input.Phi_loc_c) > 5.0)) {
          AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT = true;
        } else {
          AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT = ((rtb_DataTypeConversion3_g < 15.0) &&
            (rtb_DataTypeConversion2_f < 1.1));
        }
      } else {
        AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT = false;
      }
    } else {
      AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT = false;
    }
  } else {
    AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT = false;
  }

  if (!AutopilotStateMachine_DWork.eventTime_not_empty_m) {
    AutopilotStateMachine_DWork.eventTime_a = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty_m = true;
  }

  state_h_tmp = !AutopilotStateMachine_U.in.data.nav_valid;
  if (state_h_tmp || (!AutopilotStateMachine_U.in.data.nav_loc_valid) || ((std::abs
        (AutopilotStateMachine_U.in.data.nav_loc_error_deg) >= 0.2) ||
       ((!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT)) &&
        (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK)))) ||
      (AutopilotStateMachine_DWork.eventTime_a == 0.0)) {
    AutopilotStateMachine_DWork.eventTime_a = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_cLAND = ((AutopilotStateMachine_U.in.data.H_radio_ft <= 400.0) &&
               ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK) ||
                (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND)) &&
               ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK) ||
                (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_LAND)));
  rtb_cFLARE = ((AutopilotStateMachine_U.in.data.H_radio_ft * 15.0 <= std::abs(rtb_DataTypeConversion_h)) &&
                (((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND) ||
                  (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE)) &&
                 ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_LAND) ||
                  (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FLARE))));
  AutopilotStateMachine_DWork.state = (((rtb_on_ground != 0) && ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
    lateral_mode_FLARE) || (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_ROLL_OUT)) &&
    ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FLARE) ||
     (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_ROLL_OUT))) ||
    AutopilotStateMachine_DWork.state);
  AutopilotStateMachine_DWork.state = ((AutopilotStateMachine_U.in.data.V_gnd_kn > 50.0) &&
    ((!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_FLARE)) ||
     (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_ROLL_OUT))) &&
    ((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_FLARE)) ||
     (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ROLL_OUT))) &&
    AutopilotStateMachine_DWork.state);
  throttleCondition = ((AutopilotStateMachine_U.in.data.throttle_lever_1_pos == 45.0) ||
                       (AutopilotStateMachine_U.in.data.throttle_lever_2_pos == 45.0));
  rtb_cGA = ((!AutopilotStateMachine_DWork.sThrottleCondition) && throttleCondition &&
             (AutopilotStateMachine_U.in.data.flaps_handle_index >= 1.0) && ((rtb_on_ground == 0) || (rtb_y_p < 30.0)) &&
             (AutopilotStateMachine_U.in.data.flight_phase >= 2.0) && (AutopilotStateMachine_U.in.data.flight_phase <=
              6.0) && (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_GA_TRACK) &&
             (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS) &&
             (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS_GA));
  AutopilotStateMachine_DWork.sThrottleCondition = throttleCondition;
  if ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_CLB) ||
      (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_OP_CLB) ||
      (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_DES) ||
      (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_OP_DES)) {
    throttleCondition = true;
  } else {
    guard1 = false;
    if (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_VS) {
      u = AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft;
      if (u < 0.0) {
        u = -1.0;
      } else {
        if (u > 0.0) {
          u = 1.0;
        }
      }

      if (AutopilotStateMachine_U.in.input.H_dot_fcu_fpm < 0.0) {
        rtb_DataTypeConversion3_g = -1.0;
      } else if (AutopilotStateMachine_U.in.input.H_dot_fcu_fpm > 0.0) {
        rtb_DataTypeConversion3_g = 1.0;
      } else {
        rtb_DataTypeConversion3_g = AutopilotStateMachine_U.in.input.H_dot_fcu_fpm;
      }

      if (u == rtb_DataTypeConversion3_g) {
        throttleCondition = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FPA) {
        u = AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft;
        if (u < 0.0) {
          u = -1.0;
        } else {
          if (u > 0.0) {
            u = 1.0;
          }
        }

        if (AutopilotStateMachine_U.in.input.FPA_fcu_deg < 0.0) {
          rtb_DataTypeConversion3_g = -1.0;
        } else if (AutopilotStateMachine_U.in.input.FPA_fcu_deg > 0.0) {
          rtb_DataTypeConversion3_g = 1.0;
        } else {
          rtb_DataTypeConversion3_g = AutopilotStateMachine_U.in.input.FPA_fcu_deg;
        }

        if (u == rtb_DataTypeConversion3_g) {
          throttleCondition = true;
        } else {
          throttleCondition = (((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS) ||
                                (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS_GA)) &&
                               (AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft >
                                250.0));
        }
      } else {
        throttleCondition = (((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS) ||
                              (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS_GA)) &&
                             (AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft >
                              250.0));
      }
    }
  }

  if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_o && (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
       vertical_mode_VS) && (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_FPA)) {
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_b = 1.0;
  } else {
    if ((std::abs(AutopilotStateMachine_U.in.data.H_ind_ft - AutopilotStateMachine_U.in.input.H_fcu_ft) > 250.0) &&
        throttleCondition) {
      AutopilotStateMachine_DWork.newFcuAltitudeSelected_b = 1.0;
    }
  }

  if (((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_VS) ||
       (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FPA)) && (!throttleCondition)) {
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_b = 0.0;
  }

  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT = ((AutopilotStateMachine_U.in.data.flight_phase < 7.0) &&
    (AutopilotStateMachine_DWork.newFcuAltitudeSelected_b != 0.0) && (((!rtb_Y_j) &&
    ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_CLB) ||
     (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_DES))) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_OP_CLB) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_OP_DES) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_VS) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FPA) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS_GA)));
  switch (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode) {
   case vertical_mode_ALT_CPT:
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_b = 0.0;
    break;

   case vertical_mode_ALT:
    if (std::abs(AutopilotStateMachine_U.in.data.H_ind_ft - AutopilotStateMachine_U.in.input.H_fcu_ft) < 50.0) {
      AutopilotStateMachine_DWork.newFcuAltitudeSelected_b = 0.0;
    }
    break;
  }

  if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_o) {
    AutopilotStateMachine_DWork.newFcuAltitudeSelected = 1.0;
  } else {
    if (std::abs(AutopilotStateMachine_U.in.data.H_ind_ft - AutopilotStateMachine_U.in.input.H_fcu_ft) > 250.0) {
      AutopilotStateMachine_DWork.newFcuAltitudeSelected = 1.0;
    }
  }

  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT_CST = ((AutopilotStateMachine_U.in.data.flight_phase < 7.0)
    && (AutopilotStateMachine_DWork.newFcuAltitudeSelected != 0.0) && rtb_Y_j &&
    ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_CLB) ||
     (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_DES)));
  if (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_ALT_CPT) {
    AutopilotStateMachine_DWork.newFcuAltitudeSelected = 0.0;
  }

  throttleCondition = (((AutopilotStateMachine_U.in.data.H_radio_ft < 30.0) &&
                        ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_NONE) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS) ||
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_SRS_GA)) &&
                        (AutopilotStateMachine_U.in.data.acceleration_altitude > 0.0) &&
                        (AutopilotStateMachine_U.in.data.acceleration_altitude <
    AutopilotStateMachine_U.in.input.H_fcu_ft)) || ((AutopilotStateMachine_U.in.data.H_radio_ft >= 30.0) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_NAV) &&
    (AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft > 50.0) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ALT_CPT) &&
    ((AutopilotStateMachine_U.in.data.flight_phase == 2.0) || (AutopilotStateMachine_U.in.data.flight_phase == 6.0))));
  AutopilotStateMachine_DWork.sCLB = (throttleCondition || AutopilotStateMachine_DWork.sCLB);
  rtb_Switch_d = std::abs(AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft);
  sCLB_tmp = ((!AutopilotStateMachine_DWork.DelayInput1_DSTATE_i) && (!AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd)
              && (!AutopilotStateMachine_DWork.DelayInput1_DSTATE_a));
  AutopilotStateMachine_DWork.sCLB = (sCLB_tmp && ((AutopilotStateMachine_U.in.data.H_radio_ft < 30.0) ||
    ((AutopilotStateMachine_U.in.input.H_fcu_ft >= AutopilotStateMachine_U.in.data.H_ind_ft) && ((rtb_Switch_d >= 50.0) &&
    ((AutopilotStateMachine_U.in.input.H_fcu_ft != AutopilotStateMachine_U.in.input.H_constraint_ft) &&
     ((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ALT_CST)) ||
      (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ALT_CST_CPT)) ||
      (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS)) ||
      (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS_GA))))))) && (throttleCondition &&
    AutopilotStateMachine_DWork.sCLB));
  AutopilotStateMachine_DWork.sDES = (((rtb_on_ground == 0) && (AutopilotStateMachine_U.in.input.H_fcu_ft -
    AutopilotStateMachine_U.in.data.H_ind_ft < -50.0) && ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    vertical_mode_ALT_CST) || (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_ALT_CST_CPT)) &&
    ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_NAV) ||
     (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT) ||
     (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK))) ||
    AutopilotStateMachine_DWork.sDES);
  AutopilotStateMachine_DWork.sDES = (sCLB_tmp && ((rtb_on_ground != 0) || ((AutopilotStateMachine_U.in.input.H_fcu_ft <=
    AutopilotStateMachine_U.in.data.H_ind_ft) && ((rtb_Switch_d >= 50.0) && ((AutopilotStateMachine_U.in.input.H_fcu_ft
    != AutopilotStateMachine_U.in.input.H_constraint_ft) && ((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
    vertical_mode_ALT_CST)) || (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ALT_CST_CPT))) &&
    ((!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_NAV)) ||
     (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_CPT)) ||
     (!(AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_TRACK))))))) &&
    AutopilotStateMachine_DWork.sDES);
  throttleCondition = ((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_CPT)) &&
                       (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK)) &&
                       (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_LAND)) &&
                       (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_FLARE)));
  AutopilotStateMachine_DWork.state_j = (((AutopilotStateMachine_U.in.data.H_radio_ft > 400.0) &&
    AutopilotStateMachine_U.in.data.nav_valid && (AutopilotStateMachine_U.in.data.throttle_lever_1_pos < 45.0) &&
    (AutopilotStateMachine_U.in.data.throttle_lever_2_pos < 45.0) && rtb_BusAssignment1_input_APPR_push &&
    throttleCondition) || AutopilotStateMachine_DWork.state_j);
  AutopilotStateMachine_DWork.state_j = ((speedTargetChanged || (!AutopilotStateMachine_DWork.Delay1_DSTATE.armed.GS)) &&
    conditionSoftAlt && (rtb_BusAssignment1_input_APPR_push || (AutopilotStateMachine_DWork.Delay_DSTATE.armed.LOC ||
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT) ||
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK) ||
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND) ||
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE))) && throttleCondition &&
    (AutopilotStateMachine_U.in.data.throttle_lever_1_pos != 45.0) &&
    (AutopilotStateMachine_U.in.data.throttle_lever_2_pos != 45.0) && AutopilotStateMachine_DWork.state_j);
  rtb_DataTypeConversion3_g = std::abs(AutopilotStateMachine_U.in.data.H_dot_ft_min);
  if (rtb_Switch_d <= rtb_DataTypeConversion3_g / 5.0) {
    u = AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft;
    if (u < 0.0) {
      u = -1.0;
    } else {
      if (u > 0.0) {
        u = 1.0;
      }
    }

    if (AutopilotStateMachine_U.in.data.H_dot_ft_min < 0.0) {
      rtb_DataTypeConversion2_f = -1.0;
    } else if (AutopilotStateMachine_U.in.data.H_dot_ft_min > 0.0) {
      rtb_DataTypeConversion2_f = 1.0;
    } else {
      rtb_DataTypeConversion2_f = AutopilotStateMachine_U.in.data.H_dot_ft_min;
    }

    AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT = ((u == rtb_DataTypeConversion2_f) &&
      ((rtb_DataTypeConversion3_g >= 100.0) && ((!AutopilotStateMachine_DWork.DelayInput1_DSTATE_o) &&
      (AutopilotStateMachine_U.in.data.H_radio_ft > 400.0))));
  } else {
    AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT = false;
  }

  if ((AutopilotStateMachine_U.in.input.H_constraint_ft != 0.0) && (AutopilotStateMachine_U.in.input.H_constraint_ft !=
       AutopilotStateMachine_U.in.input.H_fcu_ft)) {
    u = AutopilotStateMachine_U.in.input.H_constraint_ft - AutopilotStateMachine_U.in.data.H_ind_ft;
    if (std::abs(u) <= std::abs(AutopilotStateMachine_U.in.data.H_dot_ft_min) / 5.0) {
      if (u < 0.0) {
        u = -1.0;
      } else {
        if (u > 0.0) {
          u = 1.0;
        }
      }

      if (AutopilotStateMachine_U.in.data.H_dot_ft_min < 0.0) {
        rtb_DataTypeConversion2_f = -1.0;
      } else if (AutopilotStateMachine_U.in.data.H_dot_ft_min > 0.0) {
        rtb_DataTypeConversion2_f = 1.0;
      } else {
        rtb_DataTypeConversion2_f = AutopilotStateMachine_U.in.data.H_dot_ft_min;
      }

      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT = ((u == rtb_DataTypeConversion2_f) &&
        ((rtb_DataTypeConversion3_g >= 300.0) && ((!AutopilotStateMachine_DWork.DelayInput1_DSTATE_o) &&
        (AutopilotStateMachine_U.in.data.H_radio_ft > 400.0))));
    } else {
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT = false;
    }
  } else {
    AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT = false;
  }

  if (AutopilotStateMachine_U.in.data.nav_valid && AutopilotStateMachine_U.in.data.nav_gs_valid &&
      ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT) ||
       (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK)) &&
      (AutopilotStateMachine_U.in.data.nav_gs_error_deg >= -0.1)) {
    rtb_DataTypeConversion3_g = std::abs(AutopilotStateMachine_U.in.data.nav_gs_error_deg);
    if ((rtb_DataTypeConversion3_g < 0.8) && rtb_FixPtRelationalOperator) {
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT = true;
    } else {
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT = (rtb_DataTypeConversion3_g < 0.4);
    }
  } else {
    AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT = false;
  }

  if (!AutopilotStateMachine_DWork.eventTime_not_empty_e) {
    AutopilotStateMachine_DWork.eventTime_c = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty_e = true;
  }

  if (state_h_tmp || (!AutopilotStateMachine_U.in.data.nav_gs_valid) || ((std::abs
        (AutopilotStateMachine_U.in.data.nav_gs_error_deg) >= 0.4) ||
       ((!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_CPT)) &&
        (!(AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK)))) ||
      (AutopilotStateMachine_DWork.eventTime_c == 0.0)) {
    AutopilotStateMachine_DWork.eventTime_c = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_DataTypeConversion3_g = std::abs(AutopilotStateMachine_U.in.data.H_ind_ft -
    AutopilotStateMachine_U.in.input.H_fcu_ft);
  AutopilotStateMachine_DWork.newFcuAltitudeSelected_c = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_o ||
    ((rtb_DataTypeConversion3_g > 250.0) || AutopilotStateMachine_DWork.newFcuAltitudeSelected_c));
  switch (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode) {
   case vertical_mode_ALT_CPT:
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_c = false;
    break;

   case vertical_mode_ALT:
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_c = ((rtb_DataTypeConversion3_g >= 50.0) &&
      AutopilotStateMachine_DWork.newFcuAltitudeSelected_c);
    break;
  }

  AutopilotStateMachine_B.BusAssignment_g.time = AutopilotStateMachine_U.in.time;
  AutopilotStateMachine_B.BusAssignment_g.data.Theta_deg = rtb_GainTheta;
  AutopilotStateMachine_B.BusAssignment_g.data.Phi_deg = rtb_GainTheta1;
  AutopilotStateMachine_B.BusAssignment_g.data.qk_deg_s = result[1];
  AutopilotStateMachine_B.BusAssignment_g.data.rk_deg_s = result[2];
  AutopilotStateMachine_B.BusAssignment_g.data.pk_deg_s = result[0];
  AutopilotStateMachine_B.BusAssignment_g.data.V_ias_kn = AutopilotStateMachine_U.in.data.V_ias_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.V_tas_kn = AutopilotStateMachine_U.in.data.V_tas_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.V_mach = AutopilotStateMachine_U.in.data.V_mach;
  AutopilotStateMachine_B.BusAssignment_g.data.V_gnd_kn = AutopilotStateMachine_U.in.data.V_gnd_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.alpha_deg = AutopilotStateMachine_U.in.data.alpha_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.H_ft = AutopilotStateMachine_U.in.data.H_ft;
  AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft = AutopilotStateMachine_U.in.data.H_ind_ft;
  AutopilotStateMachine_B.BusAssignment_g.data.H_radio_ft = AutopilotStateMachine_U.in.data.H_radio_ft;
  AutopilotStateMachine_B.BusAssignment_g.data.H_dot_ft_min = AutopilotStateMachine_U.in.data.H_dot_ft_min;
  AutopilotStateMachine_B.BusAssignment_g.data.Psi_magnetic_deg = AutopilotStateMachine_U.in.data.Psi_magnetic_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.Psi_magnetic_track_deg =
    AutopilotStateMachine_U.in.data.Psi_magnetic_track_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.Psi_true_deg = AutopilotStateMachine_U.in.data.Psi_true_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.ax_m_s2 = result_0[0];
  AutopilotStateMachine_B.BusAssignment_g.data.ay_m_s2 = result_0[1];
  AutopilotStateMachine_B.BusAssignment_g.data.az_m_s2 = result_0[2];
  AutopilotStateMachine_B.BusAssignment_g.data.bx_m_s2 = AutopilotStateMachine_U.in.data.bx_m_s2;
  AutopilotStateMachine_B.BusAssignment_g.data.by_m_s2 = AutopilotStateMachine_U.in.data.by_m_s2;
  AutopilotStateMachine_B.BusAssignment_g.data.bz_m_s2 = AutopilotStateMachine_U.in.data.bz_m_s2;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_valid = AutopilotStateMachine_U.in.data.nav_valid;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_loc_deg = AutopilotStateMachine_U.in.data.nav_loc_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_gs_deg = AutopilotStateMachine_U.in.data.nav_gs_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_dme_valid = AutopilotStateMachine_U.in.data.nav_dme_valid;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_dme_nmi = AutopilotStateMachine_U.in.data.nav_dme_nmi;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_loc_valid = AutopilotStateMachine_U.in.data.nav_loc_valid;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_loc_error_deg = AutopilotStateMachine_U.in.data.nav_loc_error_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_gs_valid = AutopilotStateMachine_U.in.data.nav_gs_valid;
  AutopilotStateMachine_B.BusAssignment_g.data.nav_gs_error_deg = AutopilotStateMachine_U.in.data.nav_gs_error_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.flight_guidance_xtk_nmi =
    AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi;
  AutopilotStateMachine_B.BusAssignment_g.data.flight_guidance_tae_deg =
    AutopilotStateMachine_U.in.data.flight_guidance_tae_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.flight_guidance_phi_deg =
    AutopilotStateMachine_U.in.data.flight_guidance_phi_deg;
  AutopilotStateMachine_B.BusAssignment_g.data.flight_phase = AutopilotStateMachine_U.in.data.flight_phase;
  AutopilotStateMachine_B.BusAssignment_g.data.V2_kn = AutopilotStateMachine_U.in.data.V2_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.VAPP_kn = AutopilotStateMachine_U.in.data.VAPP_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.VLS_kn = AutopilotStateMachine_U.in.data.VLS_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.VMAX_kn = AutopilotStateMachine_U.in.data.VMAX_kn;
  AutopilotStateMachine_B.BusAssignment_g.data.is_flight_plan_available =
    AutopilotStateMachine_U.in.data.is_flight_plan_available;
  AutopilotStateMachine_B.BusAssignment_g.data.altitude_constraint_ft =
    AutopilotStateMachine_U.in.data.altitude_constraint_ft;
  AutopilotStateMachine_B.BusAssignment_g.data.thrust_reduction_altitude =
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude;
  AutopilotStateMachine_B.BusAssignment_g.data.thrust_reduction_altitude_go_around =
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude_go_around;
  AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude =
    AutopilotStateMachine_U.in.data.acceleration_altitude;
  AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude_engine_out =
    AutopilotStateMachine_U.in.data.acceleration_altitude_engine_out;
  AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude_go_around =
    AutopilotStateMachine_U.in.data.acceleration_altitude_go_around;
  AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude_go_around_engine_out =
    AutopilotStateMachine_U.in.data.acceleration_altitude_go_around_engine_out;
  AutopilotStateMachine_B.BusAssignment_g.data.cruise_altitude = AutopilotStateMachine_U.in.data.cruise_altitude;
  AutopilotStateMachine_B.BusAssignment_g.data.on_ground = rtb_on_ground;
  AutopilotStateMachine_B.BusAssignment_g.data.zeta_deg = AutopilotStateMachine_P.Gain2_Gain *
    AutopilotStateMachine_U.in.data.zeta_pos;
  AutopilotStateMachine_B.BusAssignment_g.data.throttle_lever_1_pos =
    AutopilotStateMachine_U.in.data.throttle_lever_1_pos;
  AutopilotStateMachine_B.BusAssignment_g.data.throttle_lever_2_pos =
    AutopilotStateMachine_U.in.data.throttle_lever_2_pos;
  AutopilotStateMachine_B.BusAssignment_g.data.flaps_handle_index = AutopilotStateMachine_U.in.data.flaps_handle_index;
  AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 =
    AutopilotStateMachine_U.in.data.is_engine_operative_1;
  AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2 =
    AutopilotStateMachine_U.in.data.is_engine_operative_2;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.time_since_touchdown = rtb_y_p;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.time_since_lift_off = rtb_y_o;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.time_since_SRS = AutopilotStateMachine_U.in.time.simulation_time
    - AutopilotStateMachine_DWork.eventTime;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.H_fcu_in_selection =
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.H_constraint_valid = rtb_Y_j;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.Psi_fcu_in_selection = rtb_AND;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.gs_convergent_towards_beam = rtb_FixPtRelationalOperator;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.H_dot_radio_fpm = rtb_DataTypeConversion_h;
  AutopilotStateMachine_B.BusAssignment_g.data_computed.V_fcu_in_selection =
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_h && AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn);
  AutopilotStateMachine_B.BusAssignment_g.data_computed.ALT_soft_mode = AutopilotStateMachine_DWork.stateSoftAlt;
  AutopilotStateMachine_B.BusAssignment_g.input.FD_active = AutopilotStateMachine_U.in.input.FD_active;
  AutopilotStateMachine_B.BusAssignment_g.input.AP_1_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_p;
  AutopilotStateMachine_B.BusAssignment_g.input.AP_2_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_b;
  AutopilotStateMachine_B.BusAssignment_g.input.AP_DISCONNECT_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_d;
  AutopilotStateMachine_B.BusAssignment_g.input.HDG_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_e;
  AutopilotStateMachine_B.BusAssignment_g.input.HDG_pull = AutopilotStateMachine_DWork.DelayInput1_DSTATE_g;
  AutopilotStateMachine_B.BusAssignment_g.input.ALT_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_f;
  AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull = AutopilotStateMachine_DWork.DelayInput1_DSTATE_i;
  AutopilotStateMachine_B.BusAssignment_g.input.VS_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd;
  AutopilotStateMachine_B.BusAssignment_g.input.VS_pull = AutopilotStateMachine_DWork.DelayInput1_DSTATE_a;
  AutopilotStateMachine_B.BusAssignment_g.input.LOC_push = rtb_BusAssignment1_input_LOC_push;
  AutopilotStateMachine_B.BusAssignment_g.input.APPR_push = rtb_BusAssignment1_input_APPR_push;
  AutopilotStateMachine_B.BusAssignment_g.input.V_fcu_kn = AutopilotStateMachine_U.in.input.V_fcu_kn;
  AutopilotStateMachine_B.BusAssignment_g.input.Psi_fcu_deg = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft = AutopilotStateMachine_U.in.input.H_fcu_ft;
  AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft = AutopilotStateMachine_U.in.input.H_constraint_ft;
  AutopilotStateMachine_B.BusAssignment_g.input.H_dot_fcu_fpm = AutopilotStateMachine_U.in.input.H_dot_fcu_fpm;
  AutopilotStateMachine_B.BusAssignment_g.input.FPA_fcu_deg = AutopilotStateMachine_U.in.input.FPA_fcu_deg;
  AutopilotStateMachine_B.BusAssignment_g.input.TRK_FPA_mode = AutopilotStateMachine_U.in.input.TRK_FPA_mode;
  AutopilotStateMachine_B.BusAssignment_g.input.DIR_TO_trigger = AutopilotStateMachine_U.in.input.DIR_TO_trigger;
  AutopilotStateMachine_B.BusAssignment_g.input.is_FLX_active = AutopilotStateMachine_U.in.input.is_FLX_active;
  AutopilotStateMachine_B.BusAssignment_g.input.Slew_trigger = AutopilotStateMachine_U.in.input.Slew_trigger;
  AutopilotStateMachine_B.BusAssignment_g.input.MACH_mode = AutopilotStateMachine_U.in.input.MACH_mode;
  AutopilotStateMachine_B.BusAssignment_g.input.ATHR_engaged = AutopilotStateMachine_U.in.input.ATHR_engaged;
  AutopilotStateMachine_B.BusAssignment_g.input.is_SPEED_managed = AutopilotStateMachine_U.in.input.is_SPEED_managed;
  AutopilotStateMachine_B.BusAssignment_g.input.FDR_event = AutopilotStateMachine_U.in.input.FDR_event;
  AutopilotStateMachine_B.BusAssignment_g.input.Phi_loc_c = AutopilotStateMachine_U.in.input.Phi_loc_c;
  AutopilotStateMachine_B.BusAssignment_g.lateral.output =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.lateral.output;
  AutopilotStateMachine_B.BusAssignment_g.lateral_previous = AutopilotStateMachine_DWork.Delay_DSTATE;
  AutopilotStateMachine_B.BusAssignment_g.vertical.output =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.vertical.output;
  AutopilotStateMachine_B.BusAssignment_g.vertical_previous = AutopilotStateMachine_DWork.Delay1_DSTATE;
  AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 = AutopilotStateMachine_DWork.sAP1;
  AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 = AutopilotStateMachine_DWork.sAP2;
  AutopilotStateMachine_B.BusAssignment_g.output.lateral_law =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_law;
  AutopilotStateMachine_B.BusAssignment_g.output.lateral_mode =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_mode;
  AutopilotStateMachine_B.BusAssignment_g.output.lateral_mode_armed =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_mode_armed;
  AutopilotStateMachine_B.BusAssignment_g.output.vertical_law =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_law;
  AutopilotStateMachine_B.BusAssignment_g.output.vertical_mode =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_mode;
  AutopilotStateMachine_B.BusAssignment_g.output.vertical_mode_armed =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_mode_armed;
  AutopilotStateMachine_B.BusAssignment_g.output.mode_reversion_lateral =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.mode_reversion_lateral;
  AutopilotStateMachine_B.BusAssignment_g.output.mode_reversion_vertical =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.mode_reversion_vertical;
  AutopilotStateMachine_B.BusAssignment_g.output.mode_reversion_TRK_FPA =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.mode_reversion_TRK_FPA;
  AutopilotStateMachine_B.BusAssignment_g.output.speed_protection_mode =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.speed_protection_mode;
  AutopilotStateMachine_B.BusAssignment_g.output.autothrust_mode =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.autothrust_mode;
  AutopilotStateMachine_B.BusAssignment_g.output.Psi_c_deg =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.Psi_c_deg;
  AutopilotStateMachine_B.BusAssignment_g.output.H_c_ft =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.H_c_ft;
  AutopilotStateMachine_B.BusAssignment_g.output.H_dot_c_fpm =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.H_dot_c_fpm;
  AutopilotStateMachine_B.BusAssignment_g.output.FPA_c_deg =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.FPA_c_deg;
  AutopilotStateMachine_B.BusAssignment_g.output.V_c_kn =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.V_c_kn;
  AutopilotStateMachine_B.BusAssignment_g.output.ALT_soft_mode_active =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.ALT_soft_mode_active;
  AutopilotStateMachine_B.BusAssignment_g.output.ALT_cruise_mode_active =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.ALT_cruise_mode_active;
  AutopilotStateMachine_B.BusAssignment_g.output.EXPED_mode_active =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.EXPED_mode_active;
  AutopilotStateMachine_B.BusAssignment_g.output.FD_disconnect =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.FD_disconnect;
  AutopilotStateMachine_B.BusAssignment_g.output.FD_connect =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.FD_connect;
  AutopilotStateMachine_B.BusAssignment_g.lateral.armed.NAV = AutopilotStateMachine_DWork.state_h;
  AutopilotStateMachine_B.BusAssignment_g.lateral.armed.LOC = AutopilotStateMachine_DWork.state_d;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.NAV = ((AutopilotStateMachine_U.in.data.H_radio_ft >= 30.0) &&
    AutopilotStateMachine_U.in.data.is_flight_plan_available && (AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi
    < 10.0));
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_TRACK = (AutopilotStateMachine_U.in.time.simulation_time
    - AutopilotStateMachine_DWork.eventTime_a >= 10.0);
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LAND = rtb_cLAND;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.FLARE = rtb_cFLARE;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.ROLL_OUT = AutopilotStateMachine_DWork.state;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.GA_TRACK = rtb_cGA;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB = AutopilotStateMachine_DWork.sCLB;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES = AutopilotStateMachine_DWork.sDES;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS = AutopilotStateMachine_DWork.state_j;
  state_h_tmp = !AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT = ((rtb_Switch_d < 20.0) && state_h_tmp);
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST =
    ((AutopilotStateMachine_U.in.input.H_constraint_ft != 0.0) && (AutopilotStateMachine_U.in.input.H_constraint_ft !=
      AutopilotStateMachine_U.in.input.H_fcu_ft) && ((std::abs(AutopilotStateMachine_U.in.input.H_constraint_ft -
        AutopilotStateMachine_U.in.data.H_ind_ft) < 20.0) && state_h_tmp));
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB = ((rtb_y_o > 5.0) &&
    (AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft > 50.0) &&
    (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_NAV) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_CPT) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_TRACK) &&
    (AutopilotStateMachine_U.in.data.flight_phase >= 2.0) && (AutopilotStateMachine_U.in.data.flight_phase != 4.0) &&
    (AutopilotStateMachine_U.in.data.flight_phase != 5.0) && (AutopilotStateMachine_U.in.data.flight_phase != 6.0));
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES = ((rtb_y_o > 5.0) &&
    (AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft < -50.0) &&
    ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_NAV) ||
     (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_CPT) ||
     (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LOC_TRACK)) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_SRS_GA) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_CPT) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_TRACK) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_LAND) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_FLARE) &&
    (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_ROLL_OUT) &&
    (AutopilotStateMachine_U.in.data.flight_phase != 1.0) && (AutopilotStateMachine_U.in.data.flight_phase != 2.0) &&
    (AutopilotStateMachine_U.in.data.flight_phase != 6.0));
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_TRACK = (AutopilotStateMachine_U.in.time.simulation_time
    - AutopilotStateMachine_DWork.eventTime_c >= 15.0);
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.LAND = rtb_cLAND;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.FLARE = rtb_cFLARE;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT = AutopilotStateMachine_DWork.state;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS = ((rtb_y_p >= 30.0) &&
    (AutopilotStateMachine_U.in.data.V2_kn > 100.0) && (AutopilotStateMachine_U.in.data.flaps_handle_index > 0.0) &&
    ((AutopilotStateMachine_U.in.input.is_FLX_active && (AutopilotStateMachine_U.in.data.throttle_lever_1_pos >= 35.0) &&
      (AutopilotStateMachine_U.in.data.throttle_lever_2_pos >= 35.0)) ||
     ((AutopilotStateMachine_U.in.data.throttle_lever_1_pos == 45.0) &&
      (AutopilotStateMachine_U.in.data.throttle_lever_2_pos == 45.0))));
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS_GA = rtb_cGA;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.THR_RED = (AutopilotStateMachine_U.in.data.H_ind_ft >=
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude);
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active =
    AutopilotStateMachine_DWork.newFcuAltitudeSelected_c;
  if (AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry();
  } else {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    switch (AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine) {
     case AutopilotStateMachine_IN_GA_TRK:
      if (AutopilotStateMachine_U.in.data.H_radio_ft > 100.0) {
        if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
        } else if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
          AutopilotStateMachine_NAV_entry();
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
      break;

     case AutopilotStateMachine_IN_OFF:
      if (AutopilotStateMachine_OFF_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
        AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
      } else if (AutopilotStateMachine_OFF_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
        AutopilotStateMachine_NAV_entry();
      } else if (AutopilotStateMachine_OFF_TO_RWY(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY;
        AutopilotStateMachine_RWY_entry();
      } else if (AutopilotStateMachine_OFF_TO_RWY_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
        AutopilotStateMachine_RWY_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
      } else {
        if (AutopilotStateMachine_X_TO_GA_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_GA_TRK;
          AutopilotStateMachine_GA_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
        }
      }
      break;

     default:
      if (AutopilotStateMachine_X_TO_OFF(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
        AutopilotStateMachine_OFF_entry();
      } else if (AutopilotStateMachine_X_TO_GA_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_B.out_g.mode_reversion_TRK_FPA = true;
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_GA_TRK;
        AutopilotStateMachine_GA_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
      } else {
        switch (AutopilotStateMachine_DWork.is_ON_c) {
         case AutopilotStateMachine_IN_HDG:
          if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
            AutopilotStateMachine_NAV_entry();
          } else if (AutopilotStateMachine_ON_TO_LOC(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
            AutopilotStateMachine_LOC_CPT_entry();
          } else {
            AutopilotStateMachine_HDG_during(&AutopilotStateMachine_B.BusAssignment_g);
          }
          break;

         case AutopilotStateMachine_IN_LOC:
          if (AutopilotStateMachine_B.BusAssignment_g.data.H_radio_ft > 400.0) {
            if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
              AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
            } else if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
              AutopilotStateMachine_NAV_entry();
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }
          break;

         case AutopilotStateMachine_IN_NAV:
          if (AutopilotStateMachine_NAV_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else {
            if (AutopilotStateMachine_ON_TO_LOC(&AutopilotStateMachine_B.BusAssignment_g)) {
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
              AutopilotStateMachine_LOC_CPT_entry();
            }
          }
          break;

         case AutopilotStateMachine_IN_RWY:
          if (AutopilotStateMachine_RWY_TO_RWY_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
            AutopilotStateMachine_RWY_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else {
            if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
              AutopilotStateMachine_NAV_entry();
            }
          }
          break;

         default:
          if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else {
            if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
              AutopilotStateMachine_NAV_entry();
            }
          }
          break;
        }
      }
      break;
    }

    if (guard3) {
      switch (AutopilotStateMachine_DWork.is_LOC) {
       case AutopilotStateMachine_IN_FLARE:
        if (AutopilotStateMachine_B.BusAssignment_g.lateral.condition.ROLL_OUT) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_ROLL_OUT;
          AutopilotStateMachine_ROLL_OUT_entry();
        }
        break;

       case AutopilotStateMachine_IN_LAND:
        if (AutopilotStateMachine_B.BusAssignment_g.lateral.condition.FLARE) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_FLARE;
          AutopilotStateMachine_FLARE_entry();
        }
        break;

       case AutopilotStateMachine_IN_LOC_CPT:
        if (AutopilotStateMachine_LOC_TO_X(&AutopilotStateMachine_B.BusAssignment_g)) {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
            AutopilotStateMachine_OFF_entry();
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        break;

       case AutopilotStateMachine_IN_LOC_TRACK:
        if (AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LAND) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LAND;
          AutopilotStateMachine_LAND_entry();
        } else {
          if (AutopilotStateMachine_LOC_TO_X(&AutopilotStateMachine_B.BusAssignment_g)) {
            if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
              AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
            } else {
              if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
                AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
                AutopilotStateMachine_OFF_entry();
              }
            }
          }
        }
        break;

       default:
        if (!AutopilotStateMachine_B.BusAssignment_g.lateral.condition.ROLL_OUT) {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry(&AutopilotStateMachine_B.BusAssignment_g);
          } else {
            if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry();
            }
          }
        }
        break;
      }
    }

    if (guard2) {
      if ((!AutopilotStateMachine_B.BusAssignment_g.input.FD_active) &&
          (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 == 0.0) &&
          (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 == 0.0) &&
          (!AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.FD_connect)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
        AutopilotStateMachine_OFF_entry();
      } else {
        AutopilotStateMachine_GA_TRK_during();
      }
    }

    if (guard1) {
      if (AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_TRACK) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_TRACK;
        AutopilotStateMachine_LOC_TRACK_entry();
      }
    }
  }

  AutopilotStateMachine_B.BusAssignment_g.lateral.output = AutopilotStateMachine_B.out_g;
  if (AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
    AutopilotStateMachine_OFF_entry_p();
  } else {
    guard1 = false;
    switch (AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine) {
     case AutopilotStateMachine_IN_OFF_o:
      if (AutopilotStateMachine_B.BusAssignment_g.input.FD_active &&
          (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS) {
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_SRS;
        AutopilotStateMachine_SRS_entry();
      } else if (((AutopilotStateMachine_B.BusAssignment_g.input.FD_active &&
                   (!AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.FD_disconnect)) ||
                  (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 != 0.0) ||
                  (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 != 0.0)) &&
                 ((AutopilotStateMachine_B.BusAssignment_g.data_computed.time_since_lift_off > 5.0) ||
                  ((AutopilotStateMachine_B.BusAssignment_g.data.flight_phase >= 2.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase < 7.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) &&
                   AutopilotStateMachine_B.BusAssignment_g.input.FD_active &&
                   (!AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB)))) {
        AutopilotStateMachine_B.out.mode_reversion = true;
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB) {
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES) {
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      } else if (AutopilotStateMachine_X_TO_SRS_GA()) {
        AutopilotStateMachine_B.out.FD_connect = true;
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_SRS_GA;
        AutopilotStateMachine_SRS_GA_entry();
      } else {
        AutopilotStateMachine_OFF_during();
      }
      break;

     case AutopilotStateMachine_IN_ON_p:
      AutopilotStateMachine_ON();
      break;

     default:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        rtb_DataTypeConversion3_g = AutopilotStateMachine_B.BusAssignment_g.input.H_fcu_ft -
          AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft;
        state_h_tmp = ((AutopilotStateMachine_B.BusAssignment_g.input.ALT_pull ||
                        AutopilotStateMachine_B.BusAssignment_g.input.EXPED_push) &&
                       AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active);
        if (state_h_tmp && (rtb_DataTypeConversion3_g < -50.0)) {
          AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (state_h_tmp && (rtb_DataTypeConversion3_g > 50.0)) {
          guard1 = true;
        } else {
          rtb_DataTypeConversion3_g = std::abs(AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft -
            AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
          if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
              AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
              AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
              ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) || (rtb_DataTypeConversion3_g >
                50.0))) {
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
            AutopilotStateMachine_CLB_entry();
          } else if (AutopilotStateMachine_B.BusAssignment_g.input.ALT_push &&
                     AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES &&
                     AutopilotStateMachine_B.BusAssignment_g.vertical.condition.H_fcu_active &&
                     ((AutopilotStateMachine_B.BusAssignment_g.input.H_constraint_ft == 0.0) ||
                      (rtb_DataTypeConversion3_g > 50.0))) {
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
            AutopilotStateMachine_DES_entry();
          } else if (AutopilotStateMachine_B.BusAssignment_g.input.VS_push ||
                     AutopilotStateMachine_B.BusAssignment_g.input.VS_pull) {
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
            AutopilotStateMachine_VS_entry();
          } else if ((AutopilotStateMachine_B.BusAssignment_g.data.H_radio_ft > 400.0) &&
                     (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT)) {
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
            AutopilotStateMachine_ALT_CPT_entry();
          } else if ((AutopilotStateMachine_B.BusAssignment_g.data_computed.V_fcu_in_selection &&
                      (AutopilotStateMachine_B.BusAssignment_g.data_computed.time_since_SRS > 5.0)) ||
                     ((AutopilotStateMachine_DWork.local_H_GA_init_ft <
                       AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude_go_around) &&
                      (AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft >=
                       AutopilotStateMachine_B.BusAssignment_g.data.acceleration_altitude_go_around) &&
                      AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_1 &&
                      AutopilotStateMachine_B.BusAssignment_g.data.is_engine_operative_2)) {
            guard1 = true;
          } else if ((!AutopilotStateMachine_B.BusAssignment_g.input.FD_active) &&
                     (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 == 0.0) &&
                     (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 == 0.0) &&
                     (!AutopilotStateMachine_B.BusAssignment_g.vertical_previous.output.FD_connect)) {
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF_o;
            AutopilotStateMachine_OFF_entry_p();
          } else {
            AutopilotStateMachine_SRS_GA_during();
          }
        }
      }
      break;
    }

    if (guard1) {
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON_p;
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    }
  }

  AutopilotStateMachine_DWork.Delay1_DSTATE = AutopilotStateMachine_B.BusAssignment_g.vertical;
  AutopilotStateMachine_DWork.Delay1_DSTATE.output = AutopilotStateMachine_B.out;
  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.armed.NAV),
    &rtb_GainTheta);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.armed.LOC),
    &rtb_GainTheta1);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (AutopilotStateMachine_B.BusAssignment_g.input.FD_active ||
    (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 != 0.0) ||
    (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 != 0.0));
  if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_o) {
    Double2MultiWord(std::floor(rtb_GainTheta), &tmp_0.chunks[0U], 2);
    Double2MultiWord(std::floor(rtb_GainTheta1), &tmp_1.chunks[0U], 2);
    MultiWordIor(&tmp_0.chunks[0U], &tmp_1.chunks[0U], &tmp.chunks[0U], 2);
    AutopilotStateMachine_Y.out.output.lateral_mode_armed = uMultiWord2Double(&tmp.chunks[0U], 2, 0);
  } else {
    AutopilotStateMachine_Y.out.output.lateral_mode_armed = AutopilotStateMachine_P.Constant_Value;
  }

  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT),
    &rtb_GainTheta);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT_CST),
    &rtb_GainTheta1);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (AutopilotStateMachine_B.BusAssignment_g.input.FD_active ||
    (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1 != 0.0) ||
    (AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2 != 0.0));
  if (AutopilotStateMachine_DWork.DelayInput1_DSTATE_o) {
    Double2MultiWord(std::floor(rtb_GainTheta), &tmp_4.chunks[0U], 2);
    Double2MultiWord(std::floor(rtb_GainTheta1), &tmp_5.chunks[0U], 2);
    MultiWordIor(&tmp_4.chunks[0U], &tmp_5.chunks[0U], &tmp_3.chunks[0U], 2);
    Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
      (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB), 2))), &tmp_4.chunks[0U], 2);
    MultiWordIor(&tmp_3.chunks[0U], &tmp_4.chunks[0U], &tmp_2.chunks[0U], 2);
    Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
      (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES), 3))), &tmp_3.chunks[0U], 2);
    MultiWordIor(&tmp_2.chunks[0U], &tmp_3.chunks[0U], &tmp_1.chunks[0U], 2);
    Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
      (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS), 4))), &tmp_2.chunks[0U], 2);
    MultiWordIor(&tmp_1.chunks[0U], &tmp_2.chunks[0U], &tmp_0.chunks[0U], 2);
    AutopilotStateMachine_Y.out.output.vertical_mode_armed = uMultiWord2Double(&tmp_0.chunks[0U], 2, 0);
  } else {
    AutopilotStateMachine_Y.out.output.vertical_mode_armed = AutopilotStateMachine_P.Constant_Value_a;
  }

  rtb_GainTheta1 = static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.output.mode_reversion) -
    AutopilotStateMachine_DWork.Delay_DSTATE_f;
  rtb_GainTheta = AutopilotStateMachine_P.Raising_Value * AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta1 < rtb_GainTheta) {
    rtb_GainTheta = rtb_GainTheta1;
  }

  rtb_GainTheta1 = AutopilotStateMachine_P.Falling_Value / AutopilotStateMachine_P.Debounce_Value *
    AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta > rtb_GainTheta1) {
    rtb_GainTheta1 = rtb_GainTheta;
  }

  AutopilotStateMachine_DWork.Delay_DSTATE_f += rtb_GainTheta1;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (AutopilotStateMachine_DWork.Delay_DSTATE_f !=
    AutopilotStateMachine_P.CompareToConstant_const_d);
  AutopilotStateMachine_Y.out.output.mode_reversion_lateral = AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  rtb_GainTheta1 = static_cast<real_T>(AutopilotStateMachine_B.out.mode_reversion) -
    AutopilotStateMachine_DWork.Delay_DSTATE_ls;
  rtb_GainTheta = AutopilotStateMachine_P.Raising_Value_f * AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta1 < rtb_GainTheta) {
    rtb_GainTheta = rtb_GainTheta1;
  }

  rtb_GainTheta1 = AutopilotStateMachine_P.Falling_Value_b / AutopilotStateMachine_P.Debounce_Value_a *
    AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta > rtb_GainTheta1) {
    rtb_GainTheta1 = rtb_GainTheta;
  }

  AutopilotStateMachine_DWork.Delay_DSTATE_ls += rtb_GainTheta1;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (AutopilotStateMachine_DWork.Delay_DSTATE_ls !=
    AutopilotStateMachine_P.CompareToConstant_const_j);
  AutopilotStateMachine_Y.out.output.mode_reversion_vertical = AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  rtb_GainTheta1 = static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.output.mode_reversion_TRK_FPA) -
    AutopilotStateMachine_DWork.Delay_DSTATE_e;
  rtb_GainTheta = AutopilotStateMachine_P.Raising_Value_c * AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta1 < rtb_GainTheta) {
    rtb_GainTheta = rtb_GainTheta1;
  }

  rtb_GainTheta1 = AutopilotStateMachine_P.Falling_Value_a / AutopilotStateMachine_P.Debounce_Value_j *
    AutopilotStateMachine_B.BusAssignment_g.time.dt;
  if (rtb_GainTheta > rtb_GainTheta1) {
    rtb_GainTheta1 = rtb_GainTheta;
  }

  AutopilotStateMachine_DWork.Delay_DSTATE_e += rtb_GainTheta1;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = (AutopilotStateMachine_DWork.Delay_DSTATE_e !=
    AutopilotStateMachine_P.CompareToConstant_const_da);
  AutopilotStateMachine_Y.out.time = AutopilotStateMachine_B.BusAssignment_g.time;
  AutopilotStateMachine_Y.out.data = AutopilotStateMachine_B.BusAssignment_g.data;
  AutopilotStateMachine_Y.out.data_computed = AutopilotStateMachine_B.BusAssignment_g.data_computed;
  AutopilotStateMachine_Y.out.input = AutopilotStateMachine_B.BusAssignment_g.input;
  AutopilotStateMachine_Y.out.lateral = AutopilotStateMachine_B.BusAssignment_g.lateral;
  AutopilotStateMachine_Y.out.lateral_previous = AutopilotStateMachine_B.BusAssignment_g.lateral_previous;
  AutopilotStateMachine_Y.out.vertical = AutopilotStateMachine_DWork.Delay1_DSTATE;
  AutopilotStateMachine_Y.out.vertical_previous = AutopilotStateMachine_B.BusAssignment_g.vertical_previous;
  AutopilotStateMachine_Y.out.output.enabled_AP1 = AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP1;
  AutopilotStateMachine_Y.out.output.enabled_AP2 = AutopilotStateMachine_B.BusAssignment_g.output.enabled_AP2;
  AutopilotStateMachine_Y.out.output.lateral_law = static_cast<int32_T>
    (AutopilotStateMachine_B.BusAssignment_g.lateral.output.law);
  AutopilotStateMachine_Y.out.output.lateral_mode = static_cast<int32_T>
    (AutopilotStateMachine_B.BusAssignment_g.lateral.output.mode);
  AutopilotStateMachine_Y.out.output.vertical_law = static_cast<int32_T>(AutopilotStateMachine_B.out.law);
  AutopilotStateMachine_Y.out.output.vertical_mode = static_cast<int32_T>(AutopilotStateMachine_B.out.mode);
  AutopilotStateMachine_Y.out.output.mode_reversion_TRK_FPA = AutopilotStateMachine_DWork.DelayInput1_DSTATE_o;
  AutopilotStateMachine_Y.out.output.speed_protection_mode = AutopilotStateMachine_B.out.speed_protection_mode;
  AutopilotStateMachine_Y.out.output.autothrust_mode = static_cast<int32_T>(AutopilotStateMachine_B.out.mode_autothrust);
  AutopilotStateMachine_Y.out.output.Psi_c_deg = AutopilotStateMachine_B.BusAssignment_g.lateral.output.Psi_c_deg;
  AutopilotStateMachine_Y.out.output.H_c_ft = AutopilotStateMachine_B.out.H_c_ft;
  AutopilotStateMachine_Y.out.output.H_dot_c_fpm = AutopilotStateMachine_B.out.H_dot_c_fpm;
  AutopilotStateMachine_Y.out.output.FPA_c_deg = AutopilotStateMachine_B.out.FPA_c_deg;
  AutopilotStateMachine_Y.out.output.V_c_kn = AutopilotStateMachine_B.out.V_c_kn;
  AutopilotStateMachine_Y.out.output.ALT_soft_mode_active = AutopilotStateMachine_B.out.ALT_soft_mode_active;
  AutopilotStateMachine_Y.out.output.ALT_cruise_mode_active = AutopilotStateMachine_B.out.ALT_cruise_mode_active;
  AutopilotStateMachine_Y.out.output.EXPED_mode_active = AutopilotStateMachine_B.out.EXPED_mode_active;
  AutopilotStateMachine_Y.out.output.FD_disconnect = AutopilotStateMachine_B.out.FD_disconnect;
  AutopilotStateMachine_Y.out.output.FD_connect = AutopilotStateMachine_B.out.FD_connect;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_p = AutopilotStateMachine_U.in.input.AP_1_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = AutopilotStateMachine_U.in.input.AP_2_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = AutopilotStateMachine_U.in.input.AP_DISCONNECT_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = AutopilotStateMachine_U.in.input.HDG_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_g = AutopilotStateMachine_U.in.input.HDG_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = AutopilotStateMachine_U.in.input.ALT_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = AutopilotStateMachine_U.in.input.ALT_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = AutopilotStateMachine_U.in.input.VS_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_a = AutopilotStateMachine_U.in.input.VS_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = AutopilotStateMachine_U.in.input.LOC_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = AutopilotStateMachine_U.in.input.APPR_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = AutopilotStateMachine_U.in.input.EXPED_push;
  AutopilotStateMachine_DWork.Delay_DSTATE = AutopilotStateMachine_B.BusAssignment_g.lateral;
  AutopilotStateMachine_DWork.Delay_DSTATE_l = AutopilotStateMachine_U.in.data.nav_gs_error_deg;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE = AutopilotStateMachine_DWork.Delay1_DSTATE_b;
  AutopilotStateMachine_DWork.Delay_DSTATE_b = AutopilotStateMachine_U.in.data.H_ft;
  AutopilotStateMachine_DWork.Delay_DSTATE_a = rtb_Saturation;
  AutopilotStateMachine_DWork.Delay_DSTATE_o = rtb_Saturation1;
  AutopilotStateMachine_DWork.Delay_DSTATE_ov = rtb_Gain2;
  AutopilotStateMachine_DWork.Delay_DSTATE_c5 = AutopilotStateMachine_U.in.data.H_dot_ft_min;
  for (rtb_on_ground = 0; rtb_on_ground < 99; rtb_on_ground++) {
    AutopilotStateMachine_DWork.Delay_DSTATE_d[rtb_on_ground] = AutopilotStateMachine_DWork.Delay_DSTATE_d[rtb_on_ground
      + 1];
    AutopilotStateMachine_DWork.Delay_DSTATE_c[rtb_on_ground] = AutopilotStateMachine_DWork.Delay_DSTATE_c[rtb_on_ground
      + 1];
    AutopilotStateMachine_DWork.Delay_DSTATE_d2[rtb_on_ground] =
      AutopilotStateMachine_DWork.Delay_DSTATE_d2[rtb_on_ground + 1];
  }

  AutopilotStateMachine_DWork.Delay_DSTATE_d[99] = AutopilotStateMachine_U.in.input.H_fcu_ft;
  AutopilotStateMachine_DWork.Delay_DSTATE_c[99] = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  AutopilotStateMachine_DWork.Delay_DSTATE_d2[99] = AutopilotStateMachine_U.in.input.V_fcu_kn;
}

void AutopilotStateMachineModelClass::initialize()
{
  {
    int32_T i;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_p = AutopilotStateMachine_P.DetectIncrease_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = AutopilotStateMachine_P.DetectIncrease1_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = AutopilotStateMachine_P.DetectIncrease2_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = AutopilotStateMachine_P.DetectIncrease3_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_g = AutopilotStateMachine_P.DetectIncrease4_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = AutopilotStateMachine_P.DetectIncrease5_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = AutopilotStateMachine_P.DetectIncrease6_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = AutopilotStateMachine_P.DetectIncrease7_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_a = AutopilotStateMachine_P.DetectIncrease8_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = AutopilotStateMachine_P.DetectIncrease9_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = AutopilotStateMachine_P.DetectIncrease10_vinit;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_o = AutopilotStateMachine_P.DetectIncrease11_vinit;
    AutopilotStateMachine_DWork.Delay_DSTATE = AutopilotStateMachine_P.Delay_InitialCondition;
    AutopilotStateMachine_DWork.Delay1_DSTATE = AutopilotStateMachine_P.Delay1_InitialCondition;
    AutopilotStateMachine_DWork.Delay_DSTATE_l = AutopilotStateMachine_P.Delay_InitialCondition_c;
    AutopilotStateMachine_DWork.Delay1_DSTATE_b = AutopilotStateMachine_P.Delay1_InitialCondition_l;
    AutopilotStateMachine_DWork.DelayInput1_DSTATE = AutopilotStateMachine_P.DetectDecrease_vinit;
    AutopilotStateMachine_DWork.Delay_DSTATE_b = AutopilotStateMachine_P.Delay_InitialCondition_n;
    AutopilotStateMachine_DWork.Delay1_DSTATE_f = AutopilotStateMachine_P.Delay1_InitialCondition_d;
    AutopilotStateMachine_DWork.Delay_DSTATE_a = AutopilotStateMachine_P.Delay_InitialCondition_b;
    AutopilotStateMachine_DWork.Delay1_DSTATE_i = AutopilotStateMachine_P.Delay1_InitialCondition_j;
    AutopilotStateMachine_DWork.Delay_DSTATE_o = AutopilotStateMachine_P.DiscreteDerivativeVariableTs2_InitialCondition;
    AutopilotStateMachine_DWork.Delay_DSTATE_ov = AutopilotStateMachine_P.Delay_InitialCondition_p;
    AutopilotStateMachine_DWork.Delay1_DSTATE_h = AutopilotStateMachine_P.Delay1_InitialCondition_k;
    AutopilotStateMachine_DWork.Delay_DSTATE_c5 = AutopilotStateMachine_P.Delay_InitialCondition_k;
    AutopilotStateMachine_DWork.Delay1_DSTATE_p = AutopilotStateMachine_P.Delay1_InitialCondition_n;
    for (i = 0; i < 100; i++) {
      AutopilotStateMachine_DWork.Delay_DSTATE_d[i] = AutopilotStateMachine_P.Delay_InitialCondition_i;
      AutopilotStateMachine_DWork.Delay_DSTATE_c[i] = AutopilotStateMachine_P.Delay_InitialCondition_m;
      AutopilotStateMachine_DWork.Delay_DSTATE_d2[i] = AutopilotStateMachine_P.Delay_InitialCondition_i4;
    }

    AutopilotStateMachine_DWork.Delay_DSTATE_f = AutopilotStateMachine_P.RateLimiterDynamicVariableTs_InitialCondition;
    AutopilotStateMachine_DWork.Delay_DSTATE_ls =
      AutopilotStateMachine_P.RateLimiterDynamicVariableTs_InitialCondition_d;
    AutopilotStateMachine_DWork.Delay_DSTATE_e = AutopilotStateMachine_P.RateLimiterDynamicVariableTs_InitialCondition_g;
  }
}

void AutopilotStateMachineModelClass::terminate()
{
}

AutopilotStateMachineModelClass::AutopilotStateMachineModelClass() :
  AutopilotStateMachine_B(),
  AutopilotStateMachine_DWork(),
  AutopilotStateMachine_U(),
  AutopilotStateMachine_Y()
{
}

AutopilotStateMachineModelClass::~AutopilotStateMachineModelClass()
{
}
