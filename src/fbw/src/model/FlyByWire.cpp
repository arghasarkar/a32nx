#include "FlyByWire.h"
#include "FlyByWire_private.h"
#include "look1_binlxpw.h"
#include "look2_binlxpw.h"

const uint8_T FlyByWire_IN_InAir = 1U;
const uint8_T FlyByWire_IN_OnGround = 2U;
const uint8_T FlyByWire_IN_Flying = 1U;
const uint8_T FlyByWire_IN_Landed = 2U;
const uint8_T FlyByWire_IN_Landing100ft = 3U;
const uint8_T FlyByWire_IN_Takeoff100ft = 4U;
const uint8_T FlyByWire_IN_Flare_Reduce_Theta_c = 1U;
const uint8_T FlyByWire_IN_Flare_Set_Rate = 2U;
const uint8_T FlyByWire_IN_Flare_Store_Theta_c_deg = 3U;
const uint8_T FlyByWire_IN_Flight_High = 4U;
const uint8_T FlyByWire_IN_Flight_Low = 5U;
const uint8_T FlyByWire_IN_Ground = 6U;
const uint8_T FlyByWire_IN_frozen = 1U;
const uint8_T FlyByWire_IN_running = 2U;
const uint8_T FlyByWire_IN_Flight = 1U;
const uint8_T FlyByWire_IN_FlightToGroundTransition = 2U;
const uint8_T FlyByWire_IN_Ground_p = 3U;
const uint8_T FlyByWire_IN_automatic = 1U;
const uint8_T FlyByWire_IN_manual = 2U;
const uint8_T FlyByWire_IN_reset = 3U;
const uint8_T FlyByWire_IN_tracking = 4U;
const uint8_T FlyByWire_IN_flight_clean = 1U;
const uint8_T FlyByWire_IN_flight_flaps = 2U;
const uint8_T FlyByWire_IN_ground = 3U;
const uint8_T FlyByWire_IN_OFF = 1U;
const uint8_T FlyByWire_IN_ON = 2U;
const uint8_T FlyByWire_IN_FlightMode = 1U;
const uint8_T FlyByWire_IN_GroundMode = 2U;
void FlyByWireModelClass::FlyByWire_TimeSinceCondition(real_T rtu_time, boolean_T rtu_condition, real_T *rty_y,
  rtDW_TimeSinceCondition_FlyByWire_T *localDW)
{
  if (!localDW->eventTime_not_empty) {
    localDW->eventTime = rtu_time;
    localDW->eventTime_not_empty = true;
  }

  if ((!rtu_condition) || (localDW->eventTime == 0.0)) {
    localDW->eventTime = rtu_time;
  }

  *rty_y = rtu_time - localDW->eventTime;
}

void FlyByWireModelClass::FlyByWire_ConvertToEuler(real_T rtu_Theta, real_T rtu_Phi, real_T rtu_q, real_T rtu_r, real_T
  rtu_p, real_T *rty_qk, real_T *rty_rk, real_T *rty_pk)
{
  real_T tmp[9];
  real_T result[3];
  real_T Phi;
  real_T Theta;
  real_T result_tmp;
  real_T result_tmp_0;
  int32_T i;
  Theta = 0.017453292519943295 * rtu_Theta;
  Phi = 0.017453292519943295 * rtu_Phi;
  result_tmp = std::tan(Theta);
  result_tmp_0 = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = result_tmp_0 * result_tmp;
  tmp[6] = Phi * result_tmp;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -result_tmp_0;
  tmp[2] = 0.0;
  Theta = 1.0 / std::cos(Theta);
  tmp[5] = Theta * result_tmp_0;
  tmp[8] = Theta * Phi;
  for (i = 0; i < 3; i++) {
    result[i] = tmp[i + 6] * rtu_r + (tmp[i + 3] * rtu_q + tmp[i] * rtu_p);
  }

  *rty_qk = result[1];
  *rty_rk = result[2];
  *rty_pk = result[0];
}

void FlyByWireModelClass::step()
{
  const real_T *rtb_Switch2_c_0;
  real_T rtb_BusAssignment_c_sim_data_zeta_trim_deg;
  real_T rtb_BusAssignment_c_sim_input_delta_xi_pos;
  real_T rtb_BusAssignment_c_sim_input_delta_zeta_pos;
  real_T rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg;
  real_T rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo;
  real_T rtb_BusAssignment_e_roll_law_normal_zeta_tc_yd_deg;
  real_T rtb_BusAssignment_mv_pitch_law_rotation_eta_deg;
  real_T rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  real_T rtb_BusAssignment_sim_input_delta_eta_pos;
  real_T rtb_Divide;
  real_T rtb_Divide1_cj;
  real_T rtb_Gain;
  real_T rtb_Gain1;
  real_T rtb_Gain4;
  real_T rtb_GainPhi;
  real_T rtb_GainTheta;
  real_T rtb_Gain_c;
  real_T rtb_Gain_dq;
  real_T rtb_Gain_kf;
  real_T rtb_Gain_mk;
  real_T rtb_Gainpk;
  real_T rtb_Gainpk4;
  real_T rtb_Gainqk;
  real_T rtb_LimiteriH;
  real_T rtb_Limiterxi1;
  real_T rtb_Limiterxi2;
  real_T rtb_Loaddemand;
  real_T rtb_Product;
  real_T rtb_Product3_e;
  real_T rtb_Product4;
  real_T rtb_Sum11;
  real_T rtb_Sum1_j5;
  real_T rtb_Sum1_pp;
  real_T rtb_Sum2_nh;
  real_T rtb_Sum_dp;
  real_T rtb_Switch;
  real_T rtb_Switch_j;
  real_T rtb_Switch_k;
  real_T rtb_Tsxlo;
  real_T rtb_eta_trim_deg_rate_limit_lo_deg_s;
  real_T rtb_eta_trim_deg_rate_limit_up_deg_s;
  real_T rtb_eta_trim_deg_reset_deg;
  real_T rtb_nz_limit_up_g;
  real_T rtb_pk;
  real_T rtb_qk;
  real_T rtb_qk_g;
  real_T rtb_y_f;
  real_T u0;
  real_T u0_tmp;
  int32_T rtb_alpha_floor_inhib;
  int32_T rtb_ap_special_disc;
  int32_T rtb_in_flare;
  int32_T rtb_in_flight;
  int32_T rtb_in_rotation;
  int32_T rtb_nz_limit_lo_g;
  int32_T rtb_on_ground;
  boolean_T rtb_AND_g;
  boolean_T rtb_Logic_g_idx_0_tmp;
  boolean_T rtb_LowerRelop1_c;
  boolean_T rtb_eta_trim_deg_reset;
  boolean_T rtb_eta_trim_deg_should_write;
  FlyByWire_DWork.Delay_DSTATE += FlyByWire_U.in.time.dt;
  rtb_GainTheta = FlyByWire_P.GainTheta_Gain * FlyByWire_U.in.data.Theta_deg;
  rtb_GainPhi = FlyByWire_P.GainPhi_Gain * FlyByWire_U.in.data.Phi_deg;
  rtb_Gainqk = FlyByWire_P.Gain_Gain_n * FlyByWire_U.in.data.q_rad_s * FlyByWire_P.Gainqk_Gain;
  rtb_Gain = FlyByWire_P.Gain_Gain_l * FlyByWire_U.in.data.r_rad_s;
  rtb_Gainpk = FlyByWire_P.Gain_Gain_a * FlyByWire_U.in.data.p_rad_s * FlyByWire_P.Gainpk_Gain;
  FlyByWire_ConvertToEuler(rtb_GainTheta, rtb_GainPhi, rtb_Gainqk, rtb_Gain, rtb_Gainpk, &rtb_qk,
    &FlyByWire_Y.out.sim.data.rk_deg_s, &rtb_pk);
  FlyByWire_ConvertToEuler(rtb_GainTheta, rtb_GainPhi, FlyByWire_P.Gainqk1_Gain * (FlyByWire_P.Gain_Gain_e *
    FlyByWire_U.in.data.q_dot_rad_s2), FlyByWire_P.Gain_Gain_aw * FlyByWire_U.in.data.r_dot_rad_s2,
    FlyByWire_P.Gainpk1_Gain * (FlyByWire_P.Gain_Gain_nm * FlyByWire_U.in.data.p_dot_rad_s2), &rtb_qk_g,
    &FlyByWire_Y.out.sim.data.rk_dot_deg_s2, &rtb_y_f);
  rtb_Gainpk4 = FlyByWire_P.Gainpk4_Gain * FlyByWire_U.in.data.eta_pos;
  rtb_Product3_e = FlyByWire_P.Gainpk2_Gain * FlyByWire_U.in.data.eta_trim_deg;
  rtb_LimiteriH = FlyByWire_P.Gain1_Gain_h * FlyByWire_U.in.data.gear_animation_pos_1 - FlyByWire_P.Constant_Value_g;
  if (rtb_LimiteriH > FlyByWire_P.Saturation1_UpperSat_g) {
    rtb_LimiteriH = FlyByWire_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_LimiteriH < FlyByWire_P.Saturation1_LowerSat_j) {
      rtb_LimiteriH = FlyByWire_P.Saturation1_LowerSat_j;
    }
  }

  FlyByWire_Y.out.sim.data.gear_strut_compression_1 = rtb_LimiteriH;
  u0 = FlyByWire_P.Gain2_Gain_a * FlyByWire_U.in.data.gear_animation_pos_2 - FlyByWire_P.Constant_Value_g;
  if (u0 > FlyByWire_P.Saturation2_UpperSat_b) {
    u0 = FlyByWire_P.Saturation2_UpperSat_b;
  } else {
    if (u0 < FlyByWire_P.Saturation2_LowerSat_g) {
      u0 = FlyByWire_P.Saturation2_LowerSat_g;
    }
  }

  rtb_Tsxlo = FlyByWire_P.Gaineta_Gain * FlyByWire_U.in.input.delta_eta_pos;
  rtb_BusAssignment_sim_input_delta_eta_pos = rtb_Tsxlo;
  FlyByWire_TimeSinceCondition(FlyByWire_U.in.time.simulation_time,
    FlyByWire_P.fbw_output_MATLABStruct.sim.data_computed.on_ground == 0.0, &rtb_Tsxlo,
    &FlyByWire_DWork.sf_TimeSinceCondition_m);
  if (rtb_Tsxlo <= FlyByWire_P.CompareToConstant_const) {
    rtb_Switch = FlyByWire_P.Constant_Value_jm;
  } else {
    rtb_Switch = FlyByWire_P.Constant1_Value_g;
  }

  FlyByWire_Y.out.sim.data.pk_dot_deg_s2 = rtb_y_f;
  rtb_BusAssignment_c_sim_data_zeta_trim_deg = FlyByWire_P.Gainpk3_Gain * FlyByWire_U.in.data.zeta_trim_pos;
  rtb_BusAssignment_c_sim_input_delta_xi_pos = FlyByWire_P.Gainxi_Gain * FlyByWire_U.in.input.delta_xi_pos;
  rtb_BusAssignment_c_sim_input_delta_zeta_pos = FlyByWire_P.Gainxi1_Gain * FlyByWire_U.in.input.delta_zeta_pos;
  if (FlyByWire_DWork.is_active_c1_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c1_FlyByWire = 1U;
    FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
    rtb_on_ground = 1;
  } else if (FlyByWire_DWork.is_c1_FlyByWire == 1) {
    if ((rtb_LimiteriH > 0.1) || (u0 > 0.1)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_LimiteriH == 0.0) && (u0 == 0.0)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_Tsxlo = ((FlyByWire_U.in.data.autopilot_master_on != 0.0) || (FlyByWire_U.in.data.slew_on != 0.0) ||
               (FlyByWire_U.in.data.pause_on != 0.0) || (FlyByWire_U.in.data.tracking_mode_on_override != 0.0));
  FlyByWire_TimeSinceCondition(FlyByWire_U.in.time.simulation_time, (rtb_BusAssignment_sim_input_delta_eta_pos <
    FlyByWire_P.CompareToConstant_const_l) && (FlyByWire_P.Constant_Value_jm > FlyByWire_U.in.data.alpha_deg), &rtb_y_f,
    &FlyByWire_DWork.sf_TimeSinceCondition);
  FlyByWire_DWork.Memory_PreviousInput = FlyByWire_P.Logic_table[((((rtb_y_f > FlyByWire_P.CompareToConstant1_const) ||
    (rtb_BusAssignment_sim_input_delta_eta_pos < FlyByWire_P.CompareToConstant2_const) ||
    ((FlyByWire_U.in.data.H_radio_ft < FlyByWire_P.CompareToConstant3_const) &&
     (rtb_BusAssignment_sim_input_delta_eta_pos < FlyByWire_P.CompareToConstant4_const) &&
     (FlyByWire_U.in.data.alpha_deg < rtb_Switch + FlyByWire_P.Bias_Bias))) + (static_cast<uint32_T>
    (FlyByWire_U.in.data.alpha_deg > rtb_Switch) << 1)) << 1) + FlyByWire_DWork.Memory_PreviousInput];
  rtb_y_f = FlyByWire_P.DiscreteDerivativeVariableTs_Gain * FlyByWire_U.in.data.V_ias_kn;
  rtb_Divide = (rtb_y_f - FlyByWire_DWork.Delay_DSTATE_h) / FlyByWire_U.in.time.dt;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_a;
  FlyByWire_DWork.Delay1_DSTATE = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_a - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE + (rtb_Divide + FlyByWire_DWork.Delay_DSTATE_l) * (rtb_Gain_c / rtb_Limiterxi1);
  rtb_Gain_c = FlyByWire_P.Gain_Gain_h * FlyByWire_DWork.Delay1_DSTATE;
  if (rtb_Gain_c > FlyByWire_P.Constant1_Value_k) {
    rtb_Gain_c = FlyByWire_P.Constant1_Value_k;
  } else {
    if (FlyByWire_U.in.data.flaps_handle_index >= FlyByWire_P.Constant3_Value) {
      rtb_Switch_j = FlyByWire_P.Constant_Value;
    } else {
      rtb_Switch_j = FlyByWire_P.Constant2_Value;
    }

    if (rtb_Gain_c < rtb_Switch_j) {
      rtb_Gain_c = rtb_Switch_j;
    }
  }

  rtb_Gain_c += FlyByWire_P.Constant3_Value_n;
  rtb_LowerRelop1_c = ((FlyByWire_U.in.data.V_mach < FlyByWire_P.Constant4_Value) && (FlyByWire_U.in.data.alpha_deg >
    rtb_Gain_c));
  if (FlyByWire_DWork.is_active_c15_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c15_FlyByWire = 1U;
    FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Landed;
    rtb_alpha_floor_inhib = 1;
    rtb_ap_special_disc = 0;
  } else {
    switch (FlyByWire_DWork.is_c15_FlyByWire) {
     case FlyByWire_IN_Flying:
      if (FlyByWire_U.in.data.H_radio_ft < 100.0) {
        FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Landing100ft;
        rtb_alpha_floor_inhib = 1;
        rtb_ap_special_disc = 1;
      } else {
        rtb_alpha_floor_inhib = 0;
        rtb_ap_special_disc = 0;
      }
      break;

     case FlyByWire_IN_Landed:
      if (rtb_on_ground == 0) {
        FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Takeoff100ft;
        rtb_alpha_floor_inhib = 0;
        rtb_ap_special_disc = 0;
      } else {
        rtb_alpha_floor_inhib = 1;
        rtb_ap_special_disc = 0;
      }
      break;

     case FlyByWire_IN_Landing100ft:
      if (rtb_on_ground != 0) {
        FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Landed;
        rtb_alpha_floor_inhib = 1;
        rtb_ap_special_disc = 0;
      } else if (FlyByWire_U.in.data.H_radio_ft > 100.0) {
        FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Flying;
        rtb_alpha_floor_inhib = 0;
        rtb_ap_special_disc = 0;
      } else {
        rtb_alpha_floor_inhib = 1;
        rtb_ap_special_disc = 1;
      }
      break;

     default:
      if (FlyByWire_U.in.data.H_radio_ft > 100.0) {
        FlyByWire_DWork.is_c15_FlyByWire = FlyByWire_IN_Flying;
        rtb_alpha_floor_inhib = 0;
        rtb_ap_special_disc = 0;
      } else {
        rtb_alpha_floor_inhib = 0;
        rtb_ap_special_disc = 0;
      }
      break;
    }
  }

  rtb_Logic_g_idx_0_tmp = !FlyByWire_DWork.Memory_PreviousInput;
  FlyByWire_DWork.Memory_PreviousInput_g = FlyByWire_P.Logic_table_o[((((rtb_alpha_floor_inhib != 0) ||
    ((!rtb_LowerRelop1_c) && rtb_Logic_g_idx_0_tmp)) + (static_cast<uint32_T>(rtb_LowerRelop1_c) << 1)) << 1) +
    FlyByWire_DWork.Memory_PreviousInput_g];
  rtb_Switch_j = rtb_Product3_e;
  rtb_alpha_floor_inhib = static_cast<int32_T>(rtb_Tsxlo);
  rtb_LimiteriH = look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.uDLookupTable_bp01Data,
    FlyByWire_P.uDLookupTable_tableData, 3U);
  rtb_Gain1 = FlyByWire_P.Gain1_Gain_j * rtb_BusAssignment_c_sim_input_delta_zeta_pos;
  if (rtb_Gain1 > rtb_LimiteriH) {
    rtb_Gain1 = rtb_LimiteriH;
  } else {
    rtb_LimiteriH *= FlyByWire_P.Gain2_Gain_n;
    if (rtb_Gain1 < rtb_LimiteriH) {
      rtb_Gain1 = rtb_LimiteriH;
    }
  }

  if (FlyByWire_DWork.is_active_c3_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c3_FlyByWire = 1U;
    FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
    FlyByWire_B.in_flight = 0.0;
  } else {
    switch (FlyByWire_DWork.is_c3_FlyByWire) {
     case FlyByWire_IN_Flight:
      if ((rtb_on_ground == 1) && (rtb_GainTheta < 2.5)) {
        FlyByWire_DWork.on_ground_time = FlyByWire_U.in.time.simulation_time;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_FlightToGroundTransition;
      } else {
        FlyByWire_B.in_flight = 1.0;
      }
      break;

     case FlyByWire_IN_FlightToGroundTransition:
      if (FlyByWire_U.in.time.simulation_time - FlyByWire_DWork.on_ground_time >= 5.0) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
        FlyByWire_B.in_flight = 0.0;
      } else {
        if ((rtb_on_ground == 0) || (rtb_GainTheta >= 2.5)) {
          FlyByWire_DWork.on_ground_time = 0.0;
          FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight;
          FlyByWire_B.in_flight = 1.0;
        }
      }
      break;

     default:
      if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
        FlyByWire_DWork.on_ground_time = 0.0;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight;
        FlyByWire_B.in_flight = 1.0;
      } else {
        FlyByWire_B.in_flight = 0.0;
      }
      break;
    }
  }

  if (FlyByWire_B.in_flight > FlyByWire_P.Saturation_UpperSat_er) {
    rtb_Gain_c = FlyByWire_P.Saturation_UpperSat_er;
  } else if (FlyByWire_B.in_flight < FlyByWire_P.Saturation_LowerSat_a) {
    rtb_Gain_c = FlyByWire_P.Saturation_LowerSat_a;
  } else {
    rtb_Gain_c = FlyByWire_B.in_flight;
  }

  rtb_Limiterxi1 = rtb_Gain_c - FlyByWire_DWork.Delay_DSTATE_b;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_b += rtb_Gain_kf;
  if (FlyByWire_DWork.is_active_c6_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c6_FlyByWire = 1U;
    FlyByWire_DWork.is_c6_FlyByWire = FlyByWire_IN_OFF;
    rtb_in_rotation = 0;
  } else if (FlyByWire_DWork.is_c6_FlyByWire == 1) {
    if ((FlyByWire_DWork.Delay_DSTATE_b < 1.0) && (FlyByWire_U.in.data.V_tas_kn > 70.0) &&
        ((FlyByWire_U.in.data.thrust_lever_1_pos >= 35.0) || (FlyByWire_U.in.data.thrust_lever_2_pos >= 35.0))) {
      FlyByWire_DWork.is_c6_FlyByWire = FlyByWire_IN_ON;
      rtb_in_rotation = 1;
    } else {
      rtb_in_rotation = 0;
    }
  } else {
    if ((FlyByWire_DWork.Delay_DSTATE_b == 1.0) || (FlyByWire_U.in.data.H_radio_ft > 400.0) ||
        ((FlyByWire_U.in.data.V_tas_kn < 70.0) && ((FlyByWire_U.in.data.thrust_lever_1_pos < 35.0) ||
          (FlyByWire_U.in.data.thrust_lever_2_pos < 35.0)))) {
      FlyByWire_DWork.is_c6_FlyByWire = FlyByWire_IN_OFF;
      rtb_in_rotation = 0;
    } else {
      rtb_in_rotation = 1;
    }
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_n;
  rtb_Limiterxi2 = rtb_Limiterxi1 + FlyByWire_P.Constant_Value_i;
  FlyByWire_DWork.Delay1_DSTATE_c = 1.0 / rtb_Limiterxi2 * (FlyByWire_P.Constant_Value_i - rtb_Limiterxi1) *
    FlyByWire_DWork.Delay1_DSTATE_c + (rtb_GainTheta + FlyByWire_DWork.Delay_DSTATE_g) * (rtb_Limiterxi1 /
    rtb_Limiterxi2);
  if (FlyByWire_P.ManualSwitch_CurrentSetting == 1) {
    rtb_LimiteriH = FlyByWire_P.Constant1_Value_f;
  } else {
    rtb_LimiteriH = FlyByWire_P.Constant_Value_jz;
  }

  if (FlyByWire_DWork.is_active_c2_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c2_FlyByWire = 1U;
    FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
    rtb_in_flare = 0;
    FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
    FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
  } else {
    switch (FlyByWire_DWork.is_c2_FlyByWire) {
     case FlyByWire_IN_Flare_Reduce_Theta_c:
      if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_LimiteriH == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      }
      break;

     case FlyByWire_IN_Flare_Set_Rate:
      if (FlyByWire_P.ManualSwitch1_CurrentSetting == 1) {
        rtb_Gain_c = FlyByWire_P.Constant1_Value_f;
      } else {
        rtb_Gain_c = FlyByWire_P.Constant_Value_jz;
      }

      if ((FlyByWire_U.in.data.H_radio_ft <= 30.0) || (rtb_Gain_c == 1.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Reduce_Theta_c;
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      } else if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_LimiteriH == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flare_Store_Theta_c_deg:
      if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_LimiteriH == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        FlyByWire_B.flare_Theta_c_rate_deg_s = -(FlyByWire_DWork.Delay1_DSTATE_c + 2.0) / 8.0;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Set_Rate;
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flight_High:
      if ((FlyByWire_U.in.data.H_radio_ft <= 50.0) || (rtb_LimiteriH == 1.0)) {
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Store_Theta_c_deg;
        rtb_in_flare = 1;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     case FlyByWire_IN_Flight_Low:
      if (FlyByWire_U.in.data.H_radio_ft > 50.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_High;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     default:
      if (FlyByWire_B.in_flight == 1.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE_c;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c7_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c7_FlyByWire = 1U;
    FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
    rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
    rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
    rtb_nz_limit_up_g = 2.0;
    rtb_nz_limit_lo_g = 0;
  } else {
    switch (FlyByWire_DWork.is_c7_FlyByWire) {
     case FlyByWire_IN_flight_clean:
      if (FlyByWire_U.in.data.flaps_handle_index != 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else if ((FlyByWire_B.in_flight == 0.0) && (FlyByWire_U.in.data.flaps_handle_index == 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      }
      break;

     case FlyByWire_IN_flight_flaps:
      if (FlyByWire_U.in.data.flaps_handle_index == 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;

     default:
      if ((FlyByWire_B.in_flight != 0.0) && (FlyByWire_U.in.data.flaps_handle_index == 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if ((FlyByWire_B.in_flight != 0.0) && (FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c9_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c9_FlyByWire = 1U;
    FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
    rtb_LowerRelop1_c = false;
  } else if (FlyByWire_DWork.is_c9_FlyByWire == 1) {
    if ((rtb_in_flare == 0) && (FlyByWire_U.in.data.nz_g < 1.25) && (FlyByWire_U.in.data.nz_g > 0.5) && (std::abs
         (rtb_GainPhi) <= 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
      rtb_LowerRelop1_c = false;
    } else {
      rtb_LowerRelop1_c = true;
    }
  } else {
    if ((rtb_in_flare != 0) || (FlyByWire_U.in.data.nz_g >= 1.25) || (FlyByWire_U.in.data.nz_g <= 0.5) || (std::abs
         (rtb_GainPhi) > 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_frozen;
      rtb_LowerRelop1_c = true;
    } else {
      rtb_LowerRelop1_c = false;
    }
  }

  if (FlyByWire_DWork.is_active_c8_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c8_FlyByWire = 1U;
    FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
    rtb_eta_trim_deg_reset = true;
    rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
    rtb_eta_trim_deg_should_write = false;
  } else {
    switch (FlyByWire_DWork.is_c8_FlyByWire) {
     case FlyByWire_IN_automatic:
      if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_reset;
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      } else if (rtb_Tsxlo != 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_tracking;
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = false;
      } else {
        rtb_eta_trim_deg_reset = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = true;
      }
      break;

     case FlyByWire_IN_manual:
      if (FlyByWire_B.in_flight != 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_eta_trim_deg_reset = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = false;
      }
      break;

     case FlyByWire_IN_reset:
      if ((FlyByWire_B.in_flight == 0.0) && (rtb_Product3_e == 0.0)) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = false;
      } else {
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      }
      break;

     default:
      if (rtb_Tsxlo == 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_eta_trim_deg_reset = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3_e;
        rtb_eta_trim_deg_should_write = false;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c5_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c5_FlyByWire = 1U;
    FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
    rtb_in_flight = 0;
  } else if (FlyByWire_DWork.is_c5_FlyByWire == 1) {
    if (rtb_on_ground == 1) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
      rtb_in_flight = 0;
    } else {
      rtb_in_flight = 1;
    }
  } else {
    if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_FlightMode;
      rtb_in_flight = 1;
    } else {
      rtb_in_flight = 0;
    }
  }

  if (rtb_in_flight > FlyByWire_P.Saturation_UpperSat_p) {
    rtb_LimiteriH = FlyByWire_P.Saturation_UpperSat_p;
  } else if (rtb_in_flight < FlyByWire_P.Saturation_LowerSat_h) {
    rtb_LimiteriH = FlyByWire_P.Saturation_LowerSat_h;
  } else {
    rtb_LimiteriH = rtb_in_flight;
  }

  rtb_Limiterxi1 = rtb_LimiteriH - FlyByWire_DWork.Delay_DSTATE_c;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up_k * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_f;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_c += rtb_Gain_kf;
  rtb_Sum_dp = FlyByWire_U.in.data.engine_1_thrust_lbf - FlyByWire_U.in.data.engine_2_thrust_lbf;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_l;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_gj;
  FlyByWire_DWork.Delay1_DSTATE_h = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_gj - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE_h + (rtb_Sum_dp + FlyByWire_DWork.Delay_DSTATE_lw) * (rtb_Gain_c / rtb_Limiterxi1);
  rtb_Product = FlyByWire_DWork.Delay1_DSTATE_h * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn,
    FlyByWire_P.ScheduledGain_BreakpointsForDimension1, FlyByWire_P.ScheduledGain_Table, 3U);
  rtb_BusAssignment_p_roll_data_computed_delta_xi_deg = FlyByWire_P.Gain_Gain_c *
    rtb_BusAssignment_c_sim_input_delta_xi_pos;
  if (static_cast<real_T>(FlyByWire_DWork.Memory_PreviousInput) > FlyByWire_P.Switch2_Threshold) {
    rtb_Switch2_c_0 = &FlyByWire_P.Constant1_Value_p[0];
  } else {
    rtb_Switch2_c_0 = &FlyByWire_P.Constant_Value_f[0];
  }

  rtb_Tsxlo = look1_binlxpw(rtb_GainPhi, rtb_Switch2_c_0, FlyByWire_P.BankAngleProtection_tableData, 8U);
  rtb_Limiterxi1 = FlyByWire_P.Gain1_Gain_m * rtb_BusAssignment_c_sim_input_delta_xi_pos + rtb_Tsxlo;
  if (rtb_Limiterxi1 > FlyByWire_P.Saturation_UpperSat_n) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_n;
  } else {
    if (rtb_Limiterxi1 < FlyByWire_P.Saturation_LowerSat_o) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_o;
    }
  }

  rtb_LimiteriH = rtb_Limiterxi1 * FlyByWire_DWork.Delay_DSTATE_c;
  rtb_Tsxlo = FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain * rtb_LimiteriH * FlyByWire_U.in.time.dt;
  if ((FlyByWire_DWork.Delay_DSTATE_c == 0.0) || (rtb_alpha_floor_inhib != 0) ||
      (FlyByWire_U.in.data.autopilot_custom_on != 0.0)) {
    FlyByWire_DWork.icLoad = 1U;
  }

  if (FlyByWire_DWork.icLoad != 0) {
    FlyByWire_DWork.Delay_DSTATE_ho = rtb_GainPhi - rtb_Tsxlo;
  }

  rtb_Tsxlo += FlyByWire_DWork.Delay_DSTATE_ho;
  if (rtb_Tsxlo > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    FlyByWire_DWork.Delay_DSTATE_ho = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else if (rtb_Tsxlo < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
    FlyByWire_DWork.Delay_DSTATE_ho = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
  } else {
    FlyByWire_DWork.Delay_DSTATE_ho = rtb_Tsxlo;
  }

  if (FlyByWire_U.in.data.autopilot_custom_on > FlyByWire_P.Switch_Threshold_p) {
    if (rtb_alpha_floor_inhib > FlyByWire_P.Switch1_Threshold) {
      rtb_Gain_c = rtb_GainPhi;
    } else {
      rtb_Gain_c = FlyByWire_U.in.data.autopilot_custom_Phi_c_deg;
    }
  } else {
    rtb_Gain_c = FlyByWire_DWork.Delay_DSTATE_ho;
  }

  rtb_Tsxlo = rtb_Gain_c - FlyByWire_DWork.Delay_DSTATE_lx;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs2_up * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo >= rtb_Gain_c) {
    rtb_Tsxlo = rtb_Gain_c;
  }

  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs2_lo;
  if (rtb_Tsxlo > rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  FlyByWire_DWork.Delay_DSTATE_lx += rtb_Gain_c;
  rtb_Limiterxi1 = rtb_Gain1 - FlyByWire_DWork.Delay_DSTATE_bg;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up_m * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_p;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_bg += rtb_Gain_kf;
  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_d;
  rtb_Limiterxi2 = rtb_Limiterxi1 + FlyByWire_P.Constant_Value_d;
  FlyByWire_DWork.Delay1_DSTATE_o = 1.0 / rtb_Limiterxi2 * (FlyByWire_P.Constant_Value_d - rtb_Limiterxi1) *
    FlyByWire_DWork.Delay1_DSTATE_o + (rtb_Gain + FlyByWire_DWork.Delay_DSTATE_k) * (rtb_Limiterxi1 / rtb_Limiterxi2);
  rtb_Limiterxi2 = FlyByWire_U.in.time.dt * FlyByWire_P.WashoutFilter_C1;
  rtb_Tsxlo = rtb_Limiterxi2 + FlyByWire_P.Constant_Value_e;
  rtb_Product3_e = FlyByWire_P.Constant_Value_e / rtb_Tsxlo;
  rtb_Product4 = FlyByWire_DWork.Delay_DSTATE_cb * rtb_Product3_e;
  rtb_Product3_e *= rtb_Gain;
  FlyByWire_DWork.Delay1_DSTATE_k = 1.0 / rtb_Tsxlo * (FlyByWire_P.Constant_Value_e - rtb_Limiterxi2) *
    FlyByWire_DWork.Delay1_DSTATE_k + (rtb_Product3_e - rtb_Product4);
  FlyByWire_Y.out.roll.law_normal.pk_c_deg_s = rtb_LimiteriH;
  rtb_Product4 = ((FlyByWire_P.Gain3_Gain_k * FlyByWire_DWork.Delay_DSTATE_bg + FlyByWire_DWork.Delay_DSTATE_lx) -
                  rtb_GainPhi) * FlyByWire_P.Gain2_Gain_i + FlyByWire_P.Gain1_Gain_mg * rtb_pk * FlyByWire_P.pKp_Gain;
  if (FlyByWire_U.in.data.V_tas_kn > FlyByWire_P.Saturation_UpperSat_l) {
    rtb_Gain_c = FlyByWire_P.Saturation_UpperSat_l;
  } else if (FlyByWire_U.in.data.V_tas_kn < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Gain_c = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Gain_c = FlyByWire_U.in.data.V_tas_kn;
  }

  rtb_Limiterxi1 = (FlyByWire_DWork.Delay1_DSTATE_o - std::sin(FlyByWire_P.Gain1_Gain_b *
    FlyByWire_DWork.Delay_DSTATE_lx) * FlyByWire_P.Constant2_Value_l * std::cos(FlyByWire_P.Gain1_Gain_c * rtb_GainTheta)
                    / (FlyByWire_P.Gain6_Gain * rtb_Gain_c) * FlyByWire_P.Gain_Gain_cd) * FlyByWire_P.Gain_Gain_hk;
  rtb_Tsxlo = FlyByWire_P.Gain6_Gain_k * FlyByWire_DWork.Delay1_DSTATE_k;
  if (rtb_Limiterxi1 > FlyByWire_P.Saturation1_UpperSat_h) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation1_UpperSat_h;
  } else {
    if (rtb_Limiterxi1 < FlyByWire_P.Saturation1_LowerSat_g) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Tsxlo > FlyByWire_P.Saturation2_UpperSat_e) {
    rtb_Tsxlo = FlyByWire_P.Saturation2_UpperSat_e;
  } else {
    if (rtb_Tsxlo < FlyByWire_P.Saturation2_LowerSat_gp) {
      rtb_Tsxlo = FlyByWire_P.Saturation2_LowerSat_gp;
    }
  }

  rtb_BusAssignment_e_roll_law_normal_zeta_tc_yd_deg = FlyByWire_DWork.Delay_DSTATE_c * rtb_Limiterxi1 + rtb_Tsxlo;
  rtb_Limiterxi1 = static_cast<real_T>(rtb_on_ground) - FlyByWire_DWork.Delay_DSTATE_i;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up_d * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_fw;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_i += rtb_Gain_kf;
  if (FlyByWire_DWork.Delay_DSTATE_i > FlyByWire_P.Saturation_UpperSat_d) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_d;
  } else if (FlyByWire_DWork.Delay_DSTATE_i < FlyByWire_P.Saturation_LowerSat_j) {
    rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_j;
  } else {
    rtb_Tsxlo = FlyByWire_DWork.Delay_DSTATE_i;
  }

  rtb_LimiteriH = FlyByWire_U.in.data.autopilot_custom_Beta_c_deg * rtb_Tsxlo;
  rtb_Sum1_pp = FlyByWire_P.Constant_Value_ie - rtb_Tsxlo;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_c;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_b;
  FlyByWire_DWork.Delay1_DSTATE_b = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_b - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE_b + (FlyByWire_U.in.data.beta_deg + FlyByWire_DWork.Delay_DSTATE_hy) * (rtb_Gain_c /
    rtb_Limiterxi1);
  rtb_Product3_e = FlyByWire_U.in.data.autopilot_custom_Beta_c_deg + rtb_Product;
  rtb_Tsxlo = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter1_C1;
  rtb_Gain_c = rtb_Tsxlo + FlyByWire_P.Constant_Value_i5;
  FlyByWire_DWork.Delay1_DSTATE_d = 1.0 / rtb_Gain_c * (FlyByWire_P.Constant_Value_i5 - rtb_Tsxlo) *
    FlyByWire_DWork.Delay1_DSTATE_d + (rtb_Product3_e + FlyByWire_DWork.Delay_DSTATE_ln) * (rtb_Tsxlo / rtb_Gain_c);
  rtb_AND_g = ((rtb_alpha_floor_inhib == 0) && (FlyByWire_U.in.data.autopilot_custom_on != 0.0));
  if (rtb_AND_g) {
    rtb_Tsxlo = FlyByWire_DWork.Delay1_DSTATE_d;
  } else {
    rtb_Tsxlo = FlyByWire_DWork.Delay1_DSTATE_b;
  }

  rtb_Tsxlo -= FlyByWire_DWork.Delay1_DSTATE_b;
  rtb_Gain4 = FlyByWire_P.Gain4_Gain_o * rtb_Tsxlo;
  rtb_Tsxlo = FlyByWire_P.Gain7_Gain * rtb_Tsxlo * FlyByWire_P.DiscreteTimeIntegratorVariableTs1_Gain *
    FlyByWire_U.in.time.dt;
  if (!rtb_AND_g) {
    FlyByWire_DWork.icLoad_i = 1U;
  }

  if (FlyByWire_DWork.icLoad_i != 0) {
    FlyByWire_DWork.Delay_DSTATE_gt = FlyByWire_P.fbw_output_MATLABStruct.roll.output.zeta_deg - rtb_Tsxlo;
  }

  rtb_Tsxlo += FlyByWire_DWork.Delay_DSTATE_gt;
  if (rtb_Tsxlo > FlyByWire_P.DiscreteTimeIntegratorVariableTs1_UpperLimit) {
    FlyByWire_DWork.Delay_DSTATE_gt = FlyByWire_P.DiscreteTimeIntegratorVariableTs1_UpperLimit;
  } else if (rtb_Tsxlo < FlyByWire_P.DiscreteTimeIntegratorVariableTs1_LowerLimit) {
    FlyByWire_DWork.Delay_DSTATE_gt = FlyByWire_P.DiscreteTimeIntegratorVariableTs1_LowerLimit;
  } else {
    FlyByWire_DWork.Delay_DSTATE_gt = rtb_Tsxlo;
  }

  rtb_Limiterxi1 = (rtb_Gain4 + FlyByWire_DWork.Delay_DSTATE_gt) - FlyByWire_DWork.Delay_DSTATE_n;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs1_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs1_lo;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_n += rtb_Gain_kf;
  if (FlyByWire_U.in.data.autopilot_custom_on > FlyByWire_P.Saturation_UpperSat_k) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_k;
  } else if (FlyByWire_U.in.data.autopilot_custom_on < FlyByWire_P.Saturation_LowerSat_ae) {
    rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_ae;
  } else {
    rtb_Tsxlo = FlyByWire_U.in.data.autopilot_custom_on;
  }

  rtb_Tsxlo = ((rtb_Sum1_pp * FlyByWire_DWork.Delay_DSTATE_n + rtb_LimiteriH) * rtb_Tsxlo +
               (FlyByWire_P.Constant_Value_l - rtb_Tsxlo) * FlyByWire_P.Constant3_Value_la) +
    rtb_BusAssignment_e_roll_law_normal_zeta_tc_yd_deg;
  rtb_Sum1_pp = FlyByWire_DWork.Delay_DSTATE_bg + rtb_Tsxlo;
  if (FlyByWire_U.in.data.H_radio_ft <= FlyByWire_P.CompareToConstant_const_g) {
    rtb_Tsxlo = FlyByWire_P.Constant2_Value_c;
  }

  rtb_Tsxlo = FlyByWire_P.Gain4_Gain_h * rtb_Tsxlo * FlyByWire_P.DiscreteTimeIntegratorVariableTs1_Gain_e *
    FlyByWire_U.in.time.dt;
  if ((FlyByWire_U.in.data.autopilot_custom_on == 0.0) || (rtb_alpha_floor_inhib != 0)) {
    FlyByWire_DWork.icLoad_c = 1U;
  }

  if (FlyByWire_DWork.icLoad_c != 0) {
    FlyByWire_DWork.Delay_DSTATE_f = rtb_BusAssignment_c_sim_data_zeta_trim_deg - rtb_Tsxlo;
  }

  rtb_Tsxlo += FlyByWire_DWork.Delay_DSTATE_f;
  if (rtb_Tsxlo > FlyByWire_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_e) {
    FlyByWire_DWork.Delay_DSTATE_f = FlyByWire_P.DiscreteTimeIntegratorVariableTs1_UpperLimit_e;
  } else if (rtb_Tsxlo < FlyByWire_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_o) {
    FlyByWire_DWork.Delay_DSTATE_f = FlyByWire_P.DiscreteTimeIntegratorVariableTs1_LowerLimit_o;
  } else {
    FlyByWire_DWork.Delay_DSTATE_f = rtb_Tsxlo;
  }

  rtb_Limiterxi1 = FlyByWire_DWork.Delay_DSTATE_f - FlyByWire_DWork.Delay_DSTATE_j;
  rtb_Gain_c = FlyByWire_P.Constant_Value_ba * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.Constant1_Value_a;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_j += rtb_Gain_kf;
  u0_tmp = FlyByWire_DWork.Delay_DSTATE_c + FlyByWire_U.in.data.autopilot_custom_on;
  if (u0_tmp > FlyByWire_P.Saturation1_UpperSat_e) {
    rtb_Tsxlo = FlyByWire_P.Saturation1_UpperSat_e;
  } else if (u0_tmp < FlyByWire_P.Saturation1_LowerSat_l) {
    rtb_Tsxlo = FlyByWire_P.Saturation1_LowerSat_l;
  } else {
    rtb_Tsxlo = u0_tmp;
  }

  if (rtb_Tsxlo > FlyByWire_P.Saturation_UpperSat_ll) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_ll;
  } else {
    if (rtb_Tsxlo < FlyByWire_P.Saturation_LowerSat_og) {
      rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_og;
    }
  }

  rtb_LimiteriH = rtb_Product4 * rtb_Tsxlo;
  rtb_Tsxlo = FlyByWire_P.Constant_Value_l1 - rtb_Tsxlo;
  rtb_Tsxlo *= rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  rtb_Gain4 = rtb_LimiteriH + rtb_Tsxlo;
  if (u0_tmp > FlyByWire_P.Saturation_UpperSat_eq) {
    u0_tmp = FlyByWire_P.Saturation_UpperSat_eq;
  } else {
    if (u0_tmp < FlyByWire_P.Saturation_LowerSat_n) {
      u0_tmp = FlyByWire_P.Saturation_LowerSat_n;
    }
  }

  if (u0_tmp > FlyByWire_P.Saturation_UpperSat_i) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_i;
  } else if (u0_tmp < FlyByWire_P.Saturation_LowerSat_f) {
    rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_f;
  } else {
    rtb_Tsxlo = u0_tmp;
  }

  rtb_Gain_c = rtb_Sum1_pp * rtb_Tsxlo;
  rtb_Tsxlo = FlyByWire_P.Constant_Value_f4 - rtb_Tsxlo;
  rtb_Tsxlo *= rtb_Gain1;
  u0_tmp = rtb_Gain_c + rtb_Tsxlo;
  if (rtb_Logic_g_idx_0_tmp || (!FlyByWire_DWork.frozen_eta_trim_not_empty)) {
    FlyByWire_DWork.frozen_eta_trim = rtb_Switch_j;
    FlyByWire_DWork.frozen_eta_trim_not_empty = true;
  }

  if (rtb_in_rotation > FlyByWire_P.Saturation1_UpperSat_f) {
    rtb_LimiteriH = FlyByWire_P.Saturation1_UpperSat_f;
  } else if (rtb_in_rotation < FlyByWire_P.Saturation1_LowerSat_p) {
    rtb_LimiteriH = FlyByWire_P.Saturation1_LowerSat_p;
  } else {
    rtb_LimiteriH = rtb_in_rotation;
  }

  rtb_Limiterxi1 = rtb_LimiteriH - FlyByWire_DWork.Delay_DSTATE_fj;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs1_up_n * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs1_lo_c;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_fj += rtb_Gain_kf;
  rtb_Limiterxi1 = FlyByWire_B.flare_Theta_c_deg - FlyByWire_DWork.Delay_DSTATE_d;
  rtb_Gain_c = std::abs(FlyByWire_B.flare_Theta_c_rate_deg_s) * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_B.flare_Theta_c_rate_deg_s;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_d += rtb_Gain_kf;
  if (static_cast<real_T>(FlyByWire_DWork.Memory_PreviousInput) > FlyByWire_P.Switch_Threshold_h) {
    rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo = FlyByWire_DWork.frozen_eta_trim;
  } else {
    rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo = FlyByWire_P.Constant3_Value_l;
  }

  rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg = FlyByWire_P.Gain_Gain_d *
    rtb_BusAssignment_sim_input_delta_eta_pos;
  rtb_Tsxlo = std::cos(FlyByWire_P.Gain1_Gain_p * rtb_GainTheta);
  rtb_Divide1_cj = rtb_Tsxlo / std::cos(FlyByWire_P.Gain1_Gain_pa * rtb_GainPhi);
  rtb_LimiteriH = FlyByWire_P.Gain1_Gain_jc * rtb_qk * (FlyByWire_P.Gain_Gain_dc * FlyByWire_P.Vm_currentms_Value) +
    (FlyByWire_U.in.data.nz_g - rtb_Divide1_cj);
  if (rtb_GainPhi > FlyByWire_P.Saturation_UpperSat_dm) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_dm;
  } else if (rtb_GainPhi < FlyByWire_P.Saturation_LowerSat_pr) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_pr;
  } else {
    rtb_Limiterxi1 = rtb_GainPhi;
  }

  rtb_Limiterxi2 = rtb_Tsxlo / std::cos(FlyByWire_P.Gain1_Gain_bx * rtb_Limiterxi1);
  rtb_Limiterxi1 = FlyByWire_U.in.data.autopilot_custom_Theta_c_deg - FlyByWire_DWork.Delay_DSTATE_e;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs1_up_k * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs1_lo_h;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_e += rtb_Gain_kf;
  rtb_Limiterxi1 = rtb_BusAssignment_sim_input_delta_eta_pos - FlyByWire_DWork.Delay_DSTATE_eq;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up_f * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_fm;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_eq += rtb_Gain_kf;
  if (FlyByWire_U.in.data.autopilot_custom_on > FlyByWire_P.Switch1_Threshold_k) {
    rtb_Tsxlo = (FlyByWire_DWork.Delay_DSTATE_e - rtb_GainTheta) * FlyByWire_P.Gain4_Gain;
  } else {
    rtb_Tsxlo = FlyByWire_P.Gain1_Gain * rtb_GainTheta;
    rtb_Sum1_j5 = rtb_Divide1_cj - rtb_Limiterxi2;
    rtb_Loaddemand = look1_binlxpw(FlyByWire_DWork.Delay_DSTATE_eq, FlyByWire_P.Loaddemand_bp01Data,
      FlyByWire_P.Loaddemand_tableData, 2U);
    if (rtb_in_flare > FlyByWire_P.Switch_Threshold) {
      rtb_Switch_k = (FlyByWire_DWork.Delay_DSTATE_d - rtb_GainTheta) * FlyByWire_P.Gain_Gain;
      if (rtb_Switch_k > FlyByWire_P.Saturation_UpperSat) {
        rtb_Switch_k = FlyByWire_P.Saturation_UpperSat;
      } else {
        if (rtb_Switch_k < FlyByWire_P.Saturation_LowerSat) {
          rtb_Switch_k = FlyByWire_P.Saturation_LowerSat;
        }
      }
    } else {
      rtb_Switch_k = FlyByWire_P.Constant_Value_mi;
    }

    rtb_Limiterxi1 = FlyByWire_P.Gain2_Gain * FlyByWire_P.Theta_max1_Value - rtb_Tsxlo;
    if (rtb_Limiterxi1 > FlyByWire_P.Saturation1_UpperSat) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation1_UpperSat;
    } else {
      if (rtb_Limiterxi1 < FlyByWire_P.Saturation1_LowerSat) {
        rtb_Limiterxi1 = FlyByWire_P.Saturation1_LowerSat;
      }
    }

    rtb_Limiterxi1 = look1_binlxpw(rtb_Limiterxi1, FlyByWire_P.Loaddemand1_bp01Data, FlyByWire_P.Loaddemand1_tableData,
      2U) + rtb_Sum1_j5;
    if (rtb_Loaddemand <= rtb_Limiterxi1) {
      rtb_Limiterxi1 = FlyByWire_P.Gain3_Gain * FlyByWire_P.Theta_max3_Value - rtb_Tsxlo;
      if (rtb_Limiterxi1 > FlyByWire_P.Saturation2_UpperSat) {
        rtb_Limiterxi1 = FlyByWire_P.Saturation2_UpperSat;
      } else {
        if (rtb_Limiterxi1 < FlyByWire_P.Saturation2_LowerSat) {
          rtb_Limiterxi1 = FlyByWire_P.Saturation2_LowerSat;
        }
      }

      rtb_Limiterxi1 = look1_binlxpw(rtb_Limiterxi1, FlyByWire_P.Loaddemand2_bp01Data, FlyByWire_P.Loaddemand2_tableData,
        2U) + rtb_Sum1_j5;
      if (rtb_Loaddemand >= rtb_Limiterxi1) {
        rtb_Limiterxi1 = rtb_Loaddemand;
      }
    }

    rtb_Tsxlo = rtb_Limiterxi1 + rtb_Switch_k;
  }

  rtb_Tsxlo += rtb_Limiterxi2;
  if (rtb_Tsxlo > rtb_nz_limit_up_g) {
    rtb_Tsxlo = rtb_nz_limit_up_g;
  } else {
    if (rtb_Tsxlo < rtb_nz_limit_lo_g) {
      rtb_Tsxlo = rtb_nz_limit_lo_g;
    }
  }

  rtb_Limiterxi1 = rtb_BusAssignment_sim_input_delta_eta_pos - FlyByWire_DWork.Delay_DSTATE_ee;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs2_up_b * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Gain_c) {
    rtb_Gain_c = rtb_Limiterxi1;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs2_lo_n;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_ee += rtb_Gain_kf;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.WashoutFilter_C1_j;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_l1o;
  rtb_Limiterxi2 = FlyByWire_P.Constant_Value_l1o / rtb_Limiterxi1;
  rtb_Sum1_j5 = FlyByWire_DWork.Delay_DSTATE_el * rtb_Limiterxi2;
  if (0.0 > rtb_GainTheta - 22.5) {
    rtb_Loaddemand = 0.0;
  } else {
    rtb_Loaddemand = rtb_GainTheta - 22.5;
  }

  FlyByWire_DWork.Delay_DSTATE_el = (std::abs(rtb_GainPhi) - 3.0) / 6.0;
  if (0.0 > FlyByWire_DWork.Delay_DSTATE_el) {
    FlyByWire_DWork.Delay_DSTATE_el = 0.0;
  }

  if (rtb_Loaddemand > FlyByWire_DWork.Delay_DSTATE_el) {
    FlyByWire_DWork.Delay_DSTATE_el = rtb_Loaddemand;
  }

  FlyByWire_DWork.Delay1_DSTATE_hm = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_l1o - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE_hm + (FlyByWire_DWork.Delay_DSTATE_el * rtb_Limiterxi2 - rtb_Sum1_j5);
  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter1_C1_b;
  rtb_Limiterxi2 = rtb_Limiterxi1 + FlyByWire_P.Constant_Value_h;
  FlyByWire_DWork.Delay1_DSTATE_hc = 1.0 / rtb_Limiterxi2 * (FlyByWire_P.Constant_Value_h - rtb_Limiterxi1) *
    FlyByWire_DWork.Delay1_DSTATE_hc + (FlyByWire_U.in.data.alpha_deg + FlyByWire_DWork.Delay_DSTATE_ck) *
    (rtb_Limiterxi1 / rtb_Limiterxi2);
  rtb_Sum11 = ((FlyByWire_P.Constant_Value_jm - rtb_Switch) * FlyByWire_DWork.Delay_DSTATE_ee -
               FlyByWire_DWork.Delay1_DSTATE_hm) - (FlyByWire_DWork.Delay1_DSTATE_hc - rtb_Switch);
  rtb_Limiterxi2 = FlyByWire_P.DiscreteDerivativeVariableTs1_Gain * rtb_Sum11;
  rtb_Sum1_j5 = (rtb_Limiterxi2 - FlyByWire_DWork.Delay_DSTATE_eo) / FlyByWire_U.in.time.dt;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter2_C1;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_ab;
  FlyByWire_DWork.Delay1_DSTATE_ou = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_ab - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE_ou + (rtb_Sum1_j5 + FlyByWire_DWork.Delay_DSTATE_jx) * (rtb_Gain_c / rtb_Limiterxi1);
  rtb_Loaddemand = FlyByWire_P.DiscreteDerivativeVariableTs_Gain_h * FlyByWire_U.in.data.V_ias_kn;
  rtb_Switch_k = (rtb_Loaddemand - FlyByWire_DWork.Delay_DSTATE_o) / FlyByWire_U.in.time.dt;
  rtb_Gain_c = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_d3;
  rtb_Limiterxi1 = rtb_Gain_c + FlyByWire_P.Constant_Value_k;
  FlyByWire_DWork.Delay1_DSTATE_hs = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_k - rtb_Gain_c) *
    FlyByWire_DWork.Delay1_DSTATE_hs + (rtb_Switch_k + FlyByWire_DWork.Delay_DSTATE_m) * (rtb_Gain_c / rtb_Limiterxi1);
  rtb_Gain_mk = FlyByWire_P.DiscreteDerivativeVariableTs1_Gain_b * rtb_qk;
  if (FlyByWire_U.in.data.V_tas_kn > FlyByWire_P.Saturation3_UpperSat_p) {
    rtb_Gain_c = FlyByWire_P.Saturation3_UpperSat_p;
  } else if (FlyByWire_U.in.data.V_tas_kn < FlyByWire_P.Saturation3_LowerSat_i) {
    rtb_Gain_c = FlyByWire_P.Saturation3_LowerSat_i;
  } else {
    rtb_Gain_c = FlyByWire_U.in.data.V_tas_kn;
  }

  rtb_Limiterxi1 = rtb_LimiteriH - (look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.uDLookupTable_bp01Data_j,
    FlyByWire_P.uDLookupTable_tableData_l, 4U) / (FlyByWire_P.Gain5_Gain * rtb_Gain_c) + FlyByWire_P.Bias_Bias_d) *
    (rtb_Tsxlo - rtb_Divide1_cj);
  rtb_Divide1_cj = rtb_Limiterxi1 * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.DLUT_bp01Data,
    FlyByWire_P.DLUT_tableData, 1U) * FlyByWire_P.DiscreteDerivativeVariableTs_Gain_e;
  if (FlyByWire_DWork.Memory_PreviousInput && (FlyByWire_U.in.data.autopilot_custom_on == 0.0)) {
    rtb_Limiterxi1 = (((FlyByWire_P.alpha_err_gain_Gain * rtb_Sum11 + FlyByWire_P.v_dot_gain_Gain *
                        FlyByWire_DWork.Delay1_DSTATE_hs) + FlyByWire_P.precontrol_gain_Gain *
                       FlyByWire_DWork.Delay1_DSTATE_ou) + FlyByWire_P.qk_gain_Gain * rtb_qk) +
      FlyByWire_P.qk_dot_gain_Gain * rtb_qk_g;
    if (rtb_Limiterxi1 > FlyByWire_P.Saturation3_UpperSat) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation3_UpperSat;
    } else {
      if (rtb_Limiterxi1 < FlyByWire_P.Saturation3_LowerSat) {
        rtb_Limiterxi1 = FlyByWire_P.Saturation3_LowerSat;
      }
    }
  } else {
    rtb_Limiterxi1 = ((rtb_Gain_mk - FlyByWire_DWork.Delay_DSTATE_ca) / FlyByWire_U.in.time.dt *
                      FlyByWire_P.Gain3_Gain_l + rtb_Limiterxi1 * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn,
      FlyByWire_P.PLUT_bp01Data, FlyByWire_P.PLUT_tableData, 1U)) + (rtb_Divide1_cj - FlyByWire_DWork.Delay_DSTATE_jv) /
      FlyByWire_U.in.time.dt;
    if (rtb_Limiterxi1 > FlyByWire_P.Saturation_UpperSat_j) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_j;
    } else {
      if (rtb_Limiterxi1 < FlyByWire_P.Saturation_LowerSat_c) {
        rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_c;
      }
    }
  }

  FlyByWire_Y.out.pitch.law_normal.Cstar_g = rtb_LimiteriH;
  FlyByWire_Y.out.pitch.law_normal.nz_c_g = rtb_Tsxlo;
  rtb_Tsxlo = rtb_BusAssignment_sim_input_delta_eta_pos - FlyByWire_DWork.Delay_DSTATE_mb;
  rtb_Gain_c = FlyByWire_P.RateLimiterVariableTs_up_dl * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo < rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_d;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_mb += rtb_Gain_kf;
  if (FlyByWire_DWork.Delay_DSTATE_mb > FlyByWire_P.Saturation3_UpperSat_e) {
    rtb_Gain_c = FlyByWire_P.Saturation3_UpperSat_e;
  } else if (FlyByWire_DWork.Delay_DSTATE_mb < FlyByWire_P.Saturation3_LowerSat_c) {
    rtb_Gain_c = FlyByWire_P.Saturation3_LowerSat_c;
  } else {
    rtb_Gain_c = FlyByWire_DWork.Delay_DSTATE_mb;
  }

  rtb_Tsxlo = look1_binlxpw(static_cast<real_T>(FlyByWire_U.in.data.tailstrike_protection_on) * look2_binlxpw
    (rtb_GainTheta, FlyByWire_U.in.data.H_radio_ft, FlyByWire_P.uDLookupTable_bp01Data_l,
     FlyByWire_P.uDLookupTable_bp02Data, FlyByWire_P.uDLookupTable_tableData_d, FlyByWire_P.uDLookupTable_maxIndex, 5U) *
    rtb_Gain_c + FlyByWire_DWork.Delay_DSTATE_mb, FlyByWire_P.PitchRateDemand_bp01Data,
    FlyByWire_P.PitchRateDemand_tableData, 2U);
  FlyByWire_Y.out.pitch.law_rotation.qk_c_deg_s = rtb_Tsxlo;
  rtb_Sum11 = FlyByWire_P.DiscreteDerivativeVariableTs_Gain_c * rtb_Tsxlo;
  rtb_LimiteriH = rtb_qk - rtb_Tsxlo;
  rtb_Gain_kf = FlyByWire_P.Gain_Gain_hx * rtb_LimiteriH;
  rtb_Gain_dq = FlyByWire_P.Gain1_Gain_i * rtb_LimiteriH * FlyByWire_P.DiscreteDerivativeVariableTs_Gain_b;
  rtb_Sum2_nh = FlyByWire_P.Gain5_Gain_m * rtb_qk_g + rtb_qk;
  rtb_LimiteriH = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_i;
  rtb_Gain_c = rtb_LimiteriH + FlyByWire_P.Constant_Value_jf;
  FlyByWire_DWork.Delay1_DSTATE_i = 1.0 / rtb_Gain_c * (FlyByWire_P.Constant_Value_jf - rtb_LimiteriH) *
    FlyByWire_DWork.Delay1_DSTATE_i + (rtb_Sum2_nh + FlyByWire_DWork.Delay_DSTATE_jj) * (rtb_LimiteriH / rtb_Gain_c);
  rtb_Tsxlo = (((((rtb_Gain_dq - FlyByWire_DWork.Delay_DSTATE_dd) / FlyByWire_U.in.time.dt + rtb_Gain_kf) *
                 FlyByWire_P.Gain1_Gain_a + (rtb_Sum11 - FlyByWire_DWork.Delay_DSTATE_fd) / FlyByWire_U.in.time.dt *
                 FlyByWire_P.Gain3_Gain_p) + (FlyByWire_DWork.Delay1_DSTATE_i - rtb_Tsxlo) * FlyByWire_P.Gain4_Gain_g) +
               FlyByWire_P.Gain6_Gain_f * rtb_qk_g) * FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain_l *
    FlyByWire_U.in.time.dt;
  if (((rtb_BusAssignment_sim_input_delta_eta_pos <= FlyByWire_P.Constant_Value_j) && (rtb_on_ground != 0)) ||
      (FlyByWire_DWork.Delay_DSTATE_fj == 0.0) || (rtb_alpha_floor_inhib != 0)) {
    FlyByWire_DWork.icLoad_f = 1U;
  }

  if (FlyByWire_DWork.icLoad_f != 0) {
    FlyByWire_DWork.Delay_DSTATE_els = FlyByWire_P.Constant_Value_hn - rtb_Tsxlo;
  }

  rtb_Tsxlo += FlyByWire_DWork.Delay_DSTATE_els;
  if (rtb_Tsxlo > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_c) {
    FlyByWire_DWork.Delay_DSTATE_els = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_c;
  } else if (rtb_Tsxlo < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_p) {
    FlyByWire_DWork.Delay_DSTATE_els = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_p;
  } else {
    FlyByWire_DWork.Delay_DSTATE_els = rtb_Tsxlo;
  }

  if (rtb_on_ground > FlyByWire_P.Switch_Threshold_he) {
    if (rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg > FlyByWire_P.Saturation_UpperSat_g) {
      rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_g;
    } else if (rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg < FlyByWire_P.Saturation_LowerSat_p) {
      rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_p;
    } else {
      rtb_Tsxlo = rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg;
    }
  } else {
    rtb_Tsxlo = FlyByWire_P.Constant1_Value;
  }

  rtb_BusAssignment_mv_pitch_law_rotation_eta_deg = FlyByWire_DWork.Delay_DSTATE_els + rtb_Tsxlo;
  rtb_Tsxlo = FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain_k * rtb_Limiterxi1 * FlyByWire_U.in.time.dt;
  if ((FlyByWire_DWork.Delay_DSTATE_b == 0.0) || (rtb_alpha_floor_inhib != 0)) {
    FlyByWire_DWork.icLoad_e = 1U;
  }

  if (FlyByWire_DWork.icLoad_e != 0) {
    if (FlyByWire_B.in_flight > FlyByWire_P.Switch_Threshold_d) {
      rtb_Gain_c = rtb_Gainpk4;
    } else {
      rtb_Gain_c = rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg;
    }

    FlyByWire_DWork.Delay_DSTATE_f1 = rtb_Gain_c - rtb_Tsxlo;
  }

  rtb_Tsxlo += FlyByWire_DWork.Delay_DSTATE_f1;
  if (rtb_Tsxlo > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_cf) {
    FlyByWire_DWork.Delay_DSTATE_f1 = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_cf;
  } else if (rtb_Tsxlo < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_b) {
    FlyByWire_DWork.Delay_DSTATE_f1 = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_b;
  } else {
    FlyByWire_DWork.Delay_DSTATE_f1 = rtb_Tsxlo;
  }

  if (FlyByWire_DWork.Delay_DSTATE_b > FlyByWire_P.Saturation_UpperSat_g4) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_g4;
  } else if (FlyByWire_DWork.Delay_DSTATE_b < FlyByWire_P.Saturation_LowerSat_la) {
    rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_la;
  } else {
    rtb_Tsxlo = FlyByWire_DWork.Delay_DSTATE_b;
  }

  rtb_LimiteriH = FlyByWire_DWork.Delay_DSTATE_f1 * rtb_Tsxlo;
  rtb_Gain_kf = FlyByWire_P.Constant_Value_o - rtb_Tsxlo;
  if (FlyByWire_DWork.Delay_DSTATE_fj > FlyByWire_P.Saturation_UpperSat_c) {
    rtb_Tsxlo = FlyByWire_P.Saturation_UpperSat_c;
  } else if (FlyByWire_DWork.Delay_DSTATE_fj < FlyByWire_P.Saturation_LowerSat_m) {
    rtb_Tsxlo = FlyByWire_P.Saturation_LowerSat_m;
  } else {
    rtb_Tsxlo = FlyByWire_DWork.Delay_DSTATE_fj;
  }

  rtb_Gain_c = rtb_BusAssignment_mv_pitch_law_rotation_eta_deg * rtb_Tsxlo;
  rtb_Tsxlo = FlyByWire_P.Constant_Value_ju - rtb_Tsxlo;
  rtb_Tsxlo *= rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg;
  rtb_LimiteriH += (rtb_Gain_c + rtb_Tsxlo) * rtb_Gain_kf;
  if (rtb_LowerRelop1_c == FlyByWire_P.CompareToConstant_const_h) {
    rtb_Tsxlo = FlyByWire_P.Constant_Value_m;
  } else {
    rtb_Tsxlo = FlyByWire_DWork.Delay_DSTATE_f1;
  }

  rtb_Tsxlo = FlyByWire_P.Gain_Gain_ip * rtb_Tsxlo * FlyByWire_P.DiscreteTimeIntegratorVariableTsLimit_Gain *
    FlyByWire_U.in.time.dt;
  if (rtb_eta_trim_deg_reset) {
    FlyByWire_DWork.icLoad_io = 1U;
  }

  if (FlyByWire_DWork.icLoad_io != 0) {
    FlyByWire_DWork.Delay_DSTATE_hh = rtb_eta_trim_deg_reset_deg - rtb_Tsxlo;
  }

  FlyByWire_DWork.Delay_DSTATE_hh += rtb_Tsxlo;
  if (FlyByWire_DWork.Delay_DSTATE_hh > FlyByWire_P.Constant2_Value_e) {
    FlyByWire_DWork.Delay_DSTATE_hh = FlyByWire_P.Constant2_Value_e;
  } else {
    if (FlyByWire_DWork.Delay_DSTATE_hh < rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo) {
      FlyByWire_DWork.Delay_DSTATE_hh = rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo;
    }
  }

  rtb_Tsxlo = FlyByWire_DWork.Delay_DSTATE_hh - FlyByWire_DWork.Delay_DSTATE_ea;
  rtb_Gain_c = rtb_eta_trim_deg_rate_limit_up_deg_s * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo < rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * rtb_eta_trim_deg_rate_limit_lo_deg_s;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_ea += rtb_Gain_kf;
  rtb_Tsxlo = rtb_LimiteriH - FlyByWire_DWork.Delay_DSTATE_l1;
  rtb_Gain_c = FlyByWire_P.RateLimitereta_up * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo < rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimitereta_lo;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_l1 += rtb_Gain_kf;
  rtb_Tsxlo = rtb_Gain4 - FlyByWire_DWork.Delay_DSTATE_my;
  rtb_Gain_c = FlyByWire_P.RateLimiterxi_up * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo < rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterxi_lo;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_my += rtb_Gain_kf;
  rtb_Tsxlo = u0_tmp - FlyByWire_DWork.Delay_DSTATE_de;
  rtb_Gain_c = FlyByWire_P.RateLimiterzeta_up * FlyByWire_U.in.time.dt;
  if (rtb_Tsxlo < rtb_Gain_c) {
    rtb_Gain_c = rtb_Tsxlo;
  }

  rtb_Gain_kf = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterzeta_lo;
  if (rtb_Gain_c > rtb_Gain_kf) {
    rtb_Gain_kf = rtb_Gain_c;
  }

  FlyByWire_DWork.Delay_DSTATE_de += rtb_Gain_kf;
  FlyByWire_Y.out.sim.time.dt = FlyByWire_U.in.time.dt;
  FlyByWire_Y.out.sim.time.simulation_time = FlyByWire_U.in.time.simulation_time;
  FlyByWire_Y.out.sim.time.monotonic_time = FlyByWire_DWork.Delay_DSTATE;
  FlyByWire_Y.out.sim.data.nz_g = FlyByWire_U.in.data.nz_g;
  FlyByWire_Y.out.sim.data.Theta_deg = rtb_GainTheta;
  FlyByWire_Y.out.sim.data.Phi_deg = rtb_GainPhi;
  FlyByWire_Y.out.sim.data.q_deg_s = rtb_Gainqk;
  FlyByWire_Y.out.sim.data.r_deg_s = rtb_Gain;
  FlyByWire_Y.out.sim.data.p_deg_s = rtb_Gainpk;
  FlyByWire_Y.out.sim.data.qk_deg_s = rtb_qk;
  FlyByWire_Y.out.sim.data.pk_deg_s = rtb_pk;
  FlyByWire_Y.out.sim.data.qk_dot_deg_s2 = rtb_qk_g;
  FlyByWire_Y.out.sim.data.psi_magnetic_deg = FlyByWire_U.in.data.psi_magnetic_deg;
  FlyByWire_Y.out.sim.data.psi_true_deg = FlyByWire_U.in.data.psi_true_deg;
  FlyByWire_Y.out.sim.data.eta_deg = rtb_Gainpk4;
  FlyByWire_Y.out.sim.data.eta_trim_deg = rtb_Switch_j;
  FlyByWire_Y.out.sim.data.xi_deg = FlyByWire_P.Gainpk5_Gain * FlyByWire_U.in.data.xi_pos;
  FlyByWire_Y.out.sim.data.zeta_deg = FlyByWire_P.Gainpk6_Gain * FlyByWire_U.in.data.zeta_pos;
  FlyByWire_Y.out.sim.data.zeta_trim_deg = rtb_BusAssignment_c_sim_data_zeta_trim_deg;
  FlyByWire_Y.out.sim.data.alpha_deg = FlyByWire_U.in.data.alpha_deg;
  FlyByWire_Y.out.sim.data.beta_deg = FlyByWire_U.in.data.beta_deg;
  FlyByWire_Y.out.sim.data.beta_dot_deg_s = FlyByWire_U.in.data.beta_dot_deg_s;
  FlyByWire_Y.out.sim.data.V_ias_kn = FlyByWire_U.in.data.V_ias_kn;
  FlyByWire_Y.out.sim.data.V_tas_kn = FlyByWire_U.in.data.V_tas_kn;
  FlyByWire_Y.out.sim.data.V_mach = FlyByWire_U.in.data.V_mach;
  FlyByWire_Y.out.sim.data.H_ft = FlyByWire_U.in.data.H_ft;
  FlyByWire_Y.out.sim.data.H_ind_ft = FlyByWire_U.in.data.H_ind_ft;
  FlyByWire_Y.out.sim.data.H_radio_ft = FlyByWire_U.in.data.H_radio_ft;
  FlyByWire_Y.out.sim.data.CG_percent_MAC = FlyByWire_U.in.data.CG_percent_MAC;
  FlyByWire_Y.out.sim.data.total_weight_kg = FlyByWire_U.in.data.total_weight_kg;
  rtb_Tsxlo = FlyByWire_P.Gain_Gain_i * FlyByWire_U.in.data.gear_animation_pos_0 - FlyByWire_P.Constant_Value_g;
  if (rtb_Tsxlo > FlyByWire_P.Saturation_UpperSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_UpperSat_e;
  } else if (rtb_Tsxlo < FlyByWire_P.Saturation_LowerSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_LowerSat_e;
  } else {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = rtb_Tsxlo;
  }

  FlyByWire_Y.out.sim.data.gear_strut_compression_2 = u0;
  FlyByWire_Y.out.sim.data.flaps_handle_index = FlyByWire_U.in.data.flaps_handle_index;
  FlyByWire_Y.out.sim.data.spoilers_left_pos = FlyByWire_U.in.data.spoilers_left_pos;
  FlyByWire_Y.out.sim.data.spoilers_right_pos = FlyByWire_U.in.data.spoilers_right_pos;
  FlyByWire_Y.out.sim.data.autopilot_master_on = FlyByWire_U.in.data.autopilot_master_on;
  FlyByWire_Y.out.sim.data.slew_on = FlyByWire_U.in.data.slew_on;
  FlyByWire_Y.out.sim.data.pause_on = FlyByWire_U.in.data.pause_on;
  FlyByWire_Y.out.sim.data.tracking_mode_on_override = FlyByWire_U.in.data.tracking_mode_on_override;
  FlyByWire_Y.out.sim.data.autopilot_custom_on = FlyByWire_U.in.data.autopilot_custom_on;
  FlyByWire_Y.out.sim.data.autopilot_custom_Theta_c_deg = FlyByWire_U.in.data.autopilot_custom_Theta_c_deg;
  FlyByWire_Y.out.sim.data.autopilot_custom_Phi_c_deg = FlyByWire_U.in.data.autopilot_custom_Phi_c_deg;
  FlyByWire_Y.out.sim.data.autopilot_custom_Beta_c_deg = FlyByWire_U.in.data.autopilot_custom_Beta_c_deg;
  FlyByWire_Y.out.sim.data.simulation_rate = FlyByWire_U.in.data.simulation_rate;
  FlyByWire_Y.out.sim.data.ice_structure_percent = FlyByWire_U.in.data.ice_structure_percent;
  FlyByWire_Y.out.sim.data.linear_cl_alpha_per_deg = FlyByWire_U.in.data.linear_cl_alpha_per_deg;
  FlyByWire_Y.out.sim.data.alpha_stall_deg = FlyByWire_U.in.data.alpha_stall_deg;
  FlyByWire_Y.out.sim.data.alpha_zero_lift_deg = FlyByWire_U.in.data.alpha_zero_lift_deg;
  FlyByWire_Y.out.sim.data.ambient_density_kg_per_m3 = FlyByWire_U.in.data.ambient_density_kg_per_m3;
  FlyByWire_Y.out.sim.data.ambient_pressure_mbar = FlyByWire_U.in.data.ambient_pressure_mbar;
  FlyByWire_Y.out.sim.data.ambient_temperature_celsius = FlyByWire_U.in.data.ambient_temperature_celsius;
  FlyByWire_Y.out.sim.data.ambient_wind_x_kn = FlyByWire_U.in.data.ambient_wind_x_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_y_kn = FlyByWire_U.in.data.ambient_wind_y_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_z_kn = FlyByWire_U.in.data.ambient_wind_z_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_velocity_kn = FlyByWire_U.in.data.ambient_wind_velocity_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_direction_deg = FlyByWire_U.in.data.ambient_wind_direction_deg;
  FlyByWire_Y.out.sim.data.total_air_temperature_celsius = FlyByWire_U.in.data.total_air_temperature_celsius;
  FlyByWire_Y.out.sim.data.latitude_deg = FlyByWire_U.in.data.latitude_deg;
  FlyByWire_Y.out.sim.data.longitude_deg = FlyByWire_U.in.data.longitude_deg;
  FlyByWire_Y.out.sim.data.engine_1_thrust_lbf = FlyByWire_U.in.data.engine_1_thrust_lbf;
  FlyByWire_Y.out.sim.data.engine_2_thrust_lbf = FlyByWire_U.in.data.engine_2_thrust_lbf;
  FlyByWire_Y.out.sim.data.thrust_lever_1_pos = FlyByWire_U.in.data.thrust_lever_1_pos;
  FlyByWire_Y.out.sim.data.thrust_lever_2_pos = FlyByWire_U.in.data.thrust_lever_2_pos;
  FlyByWire_Y.out.sim.data.tailstrike_protection_on = FlyByWire_U.in.data.tailstrike_protection_on;
  FlyByWire_Y.out.sim.data_computed.on_ground = rtb_on_ground;
  FlyByWire_Y.out.sim.data_computed.tracking_mode_on = rtb_alpha_floor_inhib;
  FlyByWire_Y.out.sim.data_computed.high_aoa_prot_active = FlyByWire_DWork.Memory_PreviousInput;
  FlyByWire_Y.out.sim.data_computed.alpha_floor_command = FlyByWire_DWork.Memory_PreviousInput_g;
  FlyByWire_Y.out.sim.data_computed.protection_ap_disc = (((FlyByWire_U.in.data.alpha_deg >
    FlyByWire_P.Constant_Value_jm) && (rtb_ap_special_disc != 0)) || (FlyByWire_U.in.data.alpha_deg > rtb_Switch +
    FlyByWire_P.Bias1_Bias));
  rtb_Gain_c = std::abs(FlyByWire_U.in.data.alpha_deg - FlyByWire_P.Constant2_Value_p);
  FlyByWire_Y.out.sim.data_speeds_aoa.v_alpha_max_kn = std::sqrt(rtb_Gain_c / (FlyByWire_P.Constant_Value_jm -
    FlyByWire_P.Constant2_Value_p)) * FlyByWire_U.in.data.V_ias_kn;
  FlyByWire_Y.out.sim.data_speeds_aoa.alpha_max_deg = FlyByWire_P.Constant_Value_jm;
  FlyByWire_Y.out.sim.data_speeds_aoa.v_alpha_prot_kn = std::sqrt(rtb_Gain_c / (rtb_Switch -
    FlyByWire_P.Constant2_Value_p)) * FlyByWire_U.in.data.V_ias_kn;
  FlyByWire_Y.out.sim.data_speeds_aoa.alpha_prot_deg = rtb_Switch;
  FlyByWire_Y.out.sim.data_speeds_aoa.alpha_floor_deg = FlyByWire_P.Constant3_Value_n;
  FlyByWire_Y.out.sim.input.delta_eta_pos = rtb_BusAssignment_sim_input_delta_eta_pos;
  FlyByWire_Y.out.sim.input.delta_xi_pos = rtb_BusAssignment_c_sim_input_delta_xi_pos;
  FlyByWire_Y.out.sim.input.delta_zeta_pos = rtb_BusAssignment_c_sim_input_delta_zeta_pos;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_limit_lo =
    rtb_BusAssignment_cs_pitch_data_computed_eta_trim_deg_limit_lo;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_limit_up = FlyByWire_P.Constant2_Value_e;
  FlyByWire_Y.out.pitch.data_computed.delta_eta_deg = rtb_BusAssignment_cs_pitch_data_computed_delta_eta_deg;
  FlyByWire_Y.out.pitch.data_computed.in_flight = FlyByWire_B.in_flight;
  FlyByWire_Y.out.pitch.data_computed.in_rotation = rtb_in_rotation;
  FlyByWire_Y.out.pitch.data_computed.in_flare = rtb_in_flare;
  FlyByWire_Y.out.pitch.data_computed.in_flight_gain = FlyByWire_DWork.Delay_DSTATE_b;
  FlyByWire_Y.out.pitch.data_computed.in_rotation_gain = FlyByWire_DWork.Delay_DSTATE_fj;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_up_g = rtb_nz_limit_up_g;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_lo_g = rtb_nz_limit_lo_g;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_freeze = rtb_LowerRelop1_c;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset = rtb_eta_trim_deg_reset;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset_deg = rtb_eta_trim_deg_reset_deg;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_up_deg_s = rtb_eta_trim_deg_rate_limit_up_deg_s;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_lo_deg_s = rtb_eta_trim_deg_rate_limit_lo_deg_s;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_deg = FlyByWire_DWork.Delay1_DSTATE_c;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_deg = FlyByWire_DWork.Delay_DSTATE_d;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_rate_deg_s = FlyByWire_B.flare_Theta_c_rate_deg_s;
  FlyByWire_Y.out.pitch.law_rotation.eta_deg = rtb_BusAssignment_mv_pitch_law_rotation_eta_deg;
  FlyByWire_Y.out.pitch.law_normal.eta_dot_deg_s = rtb_Limiterxi1;
  FlyByWire_Y.out.pitch.vote.eta_dot_deg_s = rtb_Limiterxi1;
  FlyByWire_Y.out.pitch.integrated.eta_deg = FlyByWire_DWork.Delay_DSTATE_f1;
  FlyByWire_Y.out.pitch.output.eta_deg = rtb_LimiteriH;
  FlyByWire_Y.out.pitch.output.eta_trim_deg = FlyByWire_DWork.Delay_DSTATE_ea;
  FlyByWire_Y.out.roll.data_computed.delta_xi_deg = rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  FlyByWire_Y.out.roll.data_computed.delta_zeta_deg = rtb_Gain1;
  FlyByWire_Y.out.roll.data_computed.in_flight = rtb_in_flight;
  FlyByWire_Y.out.roll.data_computed.in_flight_gain = FlyByWire_DWork.Delay_DSTATE_c;
  FlyByWire_Y.out.roll.data_computed.zeta_trim_deg_should_write = (FlyByWire_U.in.data.autopilot_custom_on != 0.0);
  FlyByWire_Y.out.roll.data_computed.beta_target_deg = rtb_Product;
  FlyByWire_Y.out.roll.law_normal.Phi_c_deg = FlyByWire_DWork.Delay_DSTATE_lx;
  FlyByWire_Y.out.roll.law_normal.xi_deg = rtb_Product4;
  FlyByWire_Y.out.roll.law_normal.zeta_deg = rtb_Sum1_pp;
  FlyByWire_Y.out.roll.law_normal.zeta_tc_yd_deg = rtb_BusAssignment_e_roll_law_normal_zeta_tc_yd_deg;
  FlyByWire_Y.out.roll.output.xi_deg = rtb_Gain4;
  FlyByWire_Y.out.roll.output.zeta_deg = u0_tmp;
  FlyByWire_Y.out.roll.output.zeta_trim_deg = FlyByWire_DWork.Delay_DSTATE_j;
  u0 = FlyByWire_P.Gaineta_Gain_d * FlyByWire_DWork.Delay_DSTATE_l1;
  if (u0 > FlyByWire_P.Limitereta_UpperSat) {
    FlyByWire_Y.out.output.eta_pos = FlyByWire_P.Limitereta_UpperSat;
  } else if (u0 < FlyByWire_P.Limitereta_LowerSat) {
    FlyByWire_Y.out.output.eta_pos = FlyByWire_P.Limitereta_LowerSat;
  } else {
    FlyByWire_Y.out.output.eta_pos = u0;
  }

  u0 = FlyByWire_P.GainiH_Gain * FlyByWire_DWork.Delay_DSTATE_ea;
  if (u0 > FlyByWire_P.LimiteriH_UpperSat) {
    FlyByWire_Y.out.output.eta_trim_deg = FlyByWire_P.LimiteriH_UpperSat;
  } else if (u0 < FlyByWire_P.LimiteriH_LowerSat) {
    FlyByWire_Y.out.output.eta_trim_deg = FlyByWire_P.LimiteriH_LowerSat;
  } else {
    FlyByWire_Y.out.output.eta_trim_deg = u0;
  }

  FlyByWire_Y.out.output.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  u0 = FlyByWire_P.Gainxi_Gain_n * FlyByWire_DWork.Delay_DSTATE_my;
  if (u0 > FlyByWire_P.Limiterxi_UpperSat) {
    FlyByWire_Y.out.output.xi_pos = FlyByWire_P.Limiterxi_UpperSat;
  } else if (u0 < FlyByWire_P.Limiterxi_LowerSat) {
    FlyByWire_Y.out.output.xi_pos = FlyByWire_P.Limiterxi_LowerSat;
  } else {
    FlyByWire_Y.out.output.xi_pos = u0;
  }

  u0 = FlyByWire_P.Gainxi1_Gain_e * FlyByWire_DWork.Delay_DSTATE_de;
  if (u0 > FlyByWire_P.Limiterxi1_UpperSat) {
    FlyByWire_Y.out.output.zeta_pos = FlyByWire_P.Limiterxi1_UpperSat;
  } else if (u0 < FlyByWire_P.Limiterxi1_LowerSat) {
    FlyByWire_Y.out.output.zeta_pos = FlyByWire_P.Limiterxi1_LowerSat;
  } else {
    FlyByWire_Y.out.output.zeta_pos = u0;
  }

  u0 = FlyByWire_P.Gainxi2_Gain * FlyByWire_DWork.Delay_DSTATE_j;
  if (u0 > FlyByWire_P.Limiterxi2_UpperSat) {
    FlyByWire_Y.out.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_UpperSat;
  } else if (u0 < FlyByWire_P.Limiterxi2_LowerSat) {
    FlyByWire_Y.out.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_LowerSat;
  } else {
    FlyByWire_Y.out.output.zeta_trim_pos = u0;
  }

  FlyByWire_Y.out.output.zeta_trim_pos_should_write = (FlyByWire_U.in.data.autopilot_custom_on != 0.0);
  FlyByWire_DWork.Delay_DSTATE_h = rtb_y_f;
  FlyByWire_DWork.Delay_DSTATE_l = rtb_Divide;
  FlyByWire_DWork.Delay_DSTATE_g = rtb_GainTheta;
  FlyByWire_DWork.Delay_DSTATE_lw = rtb_Sum_dp;
  FlyByWire_DWork.icLoad = 0U;
  FlyByWire_DWork.Delay_DSTATE_k = rtb_Gain;
  FlyByWire_DWork.Delay_DSTATE_cb = rtb_Gain;
  FlyByWire_DWork.Delay_DSTATE_hy = FlyByWire_U.in.data.beta_deg;
  FlyByWire_DWork.Delay_DSTATE_ln = rtb_Product3_e;
  FlyByWire_DWork.icLoad_i = 0U;
  FlyByWire_DWork.icLoad_c = 0U;
  FlyByWire_DWork.Delay_DSTATE_ck = FlyByWire_U.in.data.alpha_deg;
  FlyByWire_DWork.Delay_DSTATE_eo = rtb_Limiterxi2;
  FlyByWire_DWork.Delay_DSTATE_jx = rtb_Sum1_j5;
  FlyByWire_DWork.Delay_DSTATE_o = rtb_Loaddemand;
  FlyByWire_DWork.Delay_DSTATE_m = rtb_Switch_k;
  FlyByWire_DWork.Delay_DSTATE_ca = rtb_Gain_mk;
  FlyByWire_DWork.Delay_DSTATE_jv = rtb_Divide1_cj;
  FlyByWire_DWork.Delay_DSTATE_fd = rtb_Sum11;
  FlyByWire_DWork.Delay_DSTATE_dd = rtb_Gain_dq;
  FlyByWire_DWork.Delay_DSTATE_jj = rtb_Sum2_nh;
  FlyByWire_DWork.icLoad_f = 0U;
  FlyByWire_DWork.icLoad_e = 0U;
  FlyByWire_DWork.icLoad_io = 0U;
}

void FlyByWireModelClass::initialize()
{
  FlyByWire_DWork.Delay_DSTATE = FlyByWire_P.Delay_InitialCondition;
  FlyByWire_DWork.Memory_PreviousInput = FlyByWire_P.SRFlipFlop_initial_condition;
  FlyByWire_DWork.Delay_DSTATE_h = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_l = FlyByWire_P.Delay_InitialCondition_m;
  FlyByWire_DWork.Delay1_DSTATE = FlyByWire_P.Delay1_InitialCondition;
  FlyByWire_DWork.Memory_PreviousInput_g = FlyByWire_P.SRFlipFlop1_initial_condition;
  FlyByWire_DWork.Delay_DSTATE_b = FlyByWire_P.RateLimiterVariableTs_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_g = FlyByWire_P.Delay_InitialCondition_f;
  FlyByWire_DWork.Delay1_DSTATE_c = FlyByWire_P.Delay1_InitialCondition_n;
  FlyByWire_DWork.Delay_DSTATE_c = FlyByWire_P.RateLimiterVariableTs_InitialCondition_f;
  FlyByWire_DWork.Delay_DSTATE_lw = FlyByWire_P.Delay_InitialCondition_l;
  FlyByWire_DWork.Delay1_DSTATE_h = FlyByWire_P.Delay1_InitialCondition_b;
  FlyByWire_DWork.icLoad = 1U;
  FlyByWire_DWork.Delay_DSTATE_lx = FlyByWire_P.RateLimiterVariableTs2_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_bg = FlyByWire_P.RateLimiterVariableTs_InitialCondition_fc;
  FlyByWire_DWork.Delay_DSTATE_k = FlyByWire_P.Delay_InitialCondition_a;
  FlyByWire_DWork.Delay1_DSTATE_o = FlyByWire_P.Delay1_InitialCondition_e;
  FlyByWire_DWork.Delay_DSTATE_cb = FlyByWire_P.Delay_InitialCondition_c;
  FlyByWire_DWork.Delay1_DSTATE_k = FlyByWire_P.Delay1_InitialCondition_d;
  FlyByWire_DWork.Delay_DSTATE_i = FlyByWire_P.RateLimiterVariableTs_InitialCondition_p;
  FlyByWire_DWork.Delay_DSTATE_n = FlyByWire_P.RateLimiterVariableTs1_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_hy = FlyByWire_P.Delay_InitialCondition_j;
  FlyByWire_DWork.Delay1_DSTATE_b = FlyByWire_P.Delay1_InitialCondition_m;
  FlyByWire_DWork.Delay_DSTATE_ln = FlyByWire_P.Delay_InitialCondition_d;
  FlyByWire_DWork.Delay1_DSTATE_d = FlyByWire_P.Delay1_InitialCondition_mb;
  FlyByWire_DWork.icLoad_i = 1U;
  FlyByWire_DWork.Delay_DSTATE_j = FlyByWire_P.RateLimiterDynamicVariableTs_InitialCondition;
  FlyByWire_DWork.icLoad_c = 1U;
  FlyByWire_DWork.Delay_DSTATE_fj = FlyByWire_P.RateLimiterVariableTs1_InitialCondition_h;
  FlyByWire_DWork.Delay_DSTATE_d = FlyByWire_P.RateLimiterDynamicVariableTs_InitialCondition_n;
  FlyByWire_DWork.Delay_DSTATE_e = FlyByWire_P.RateLimiterVariableTs1_InitialCondition_hb;
  FlyByWire_DWork.Delay_DSTATE_eq = FlyByWire_P.RateLimiterVariableTs_InitialCondition_c;
  FlyByWire_DWork.Delay_DSTATE_ee = FlyByWire_P.RateLimiterVariableTs2_InitialCondition_j;
  FlyByWire_DWork.Delay_DSTATE_el = FlyByWire_P.Delay_InitialCondition_p;
  FlyByWire_DWork.Delay1_DSTATE_hm = FlyByWire_P.Delay1_InitialCondition_p;
  FlyByWire_DWork.Delay_DSTATE_ck = FlyByWire_P.Delay_InitialCondition_o;
  FlyByWire_DWork.Delay1_DSTATE_hc = FlyByWire_P.Delay1_InitialCondition_m3;
  FlyByWire_DWork.Delay_DSTATE_eo = FlyByWire_P.DiscreteDerivativeVariableTs1_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_jx = FlyByWire_P.Delay_InitialCondition_fb;
  FlyByWire_DWork.Delay1_DSTATE_ou = FlyByWire_P.Delay1_InitialCondition_o;
  FlyByWire_DWork.Delay_DSTATE_o = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition_j;
  FlyByWire_DWork.Delay_DSTATE_m = FlyByWire_P.Delay_InitialCondition_dv;
  FlyByWire_DWork.Delay1_DSTATE_hs = FlyByWire_P.Delay1_InitialCondition_h;
  FlyByWire_DWork.Delay_DSTATE_ca = FlyByWire_P.DiscreteDerivativeVariableTs1_InitialCondition_f;
  FlyByWire_DWork.Delay_DSTATE_jv = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition_c;
  FlyByWire_DWork.Delay_DSTATE_mb = FlyByWire_P.RateLimiterVariableTs_InitialCondition_n;
  FlyByWire_DWork.Delay_DSTATE_fd = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition_d;
  FlyByWire_DWork.Delay_DSTATE_dd = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition_k;
  FlyByWire_DWork.Delay_DSTATE_jj = FlyByWire_P.Delay_InitialCondition_ds;
  FlyByWire_DWork.Delay1_DSTATE_i = FlyByWire_P.Delay1_InitialCondition_mn;
  FlyByWire_DWork.icLoad_f = 1U;
  FlyByWire_DWork.icLoad_e = 1U;
  FlyByWire_DWork.Delay_DSTATE_ea = FlyByWire_P.RateLimiterDynamicVariableTs_InitialCondition_i;
  FlyByWire_DWork.icLoad_io = 1U;
  FlyByWire_DWork.Delay_DSTATE_l1 = FlyByWire_P.RateLimitereta_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_my = FlyByWire_P.RateLimiterxi_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_de = FlyByWire_P.RateLimiterzeta_InitialCondition;
}

void FlyByWireModelClass::terminate()
{
}

FlyByWireModelClass::FlyByWireModelClass() :
  FlyByWire_B(),
  FlyByWire_DWork(),
  FlyByWire_U(),
  FlyByWire_Y()
{
}

FlyByWireModelClass::~FlyByWireModelClass()
{
}
