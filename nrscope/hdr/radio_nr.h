#ifndef RADIO_H
#define RADIO_H

#include "srsran/common/band_helper.h"
#include "srsran/common/crash_handler.h"
#include "srsran/common/string_helpers.h"
#include "srsue/hdr/phy/phy_nr_sa.h"
#include "srsue/hdr/phy/nr/cell_search.h"
#include "test/phy/dummy_ue_stack.h"
#include "srsue/hdr/stack/ue_stack_nr.h"
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>
#include "srsran/asn1/rrc_nr.h"
#include "srsran/asn1/asn1_utils.h"
#include "srsran/mac/mac_rar_pdu_nr.h"

#include "nrscope/hdr/rach_uplink.h"

#include "srsran/srslog/logger.h"
#include <iostream>
#include <cmath>
#include <pthread.h>

struct cell_searcher_args_t {
  // Generic parameters
  double                      srate_hz        = 11.52e6;
  srsran_carrier_nr_t         base_carrier    = SRSRAN_DEFAULT_CARRIER_NR;
  srsran_ssb_pattern_t        ssb_pattern;
  srsran_subcarrier_spacing_t ssb_scs;
  srsran_duplex_mode_t        duplex_mode     = SRSRAN_DUPLEX_MODE_TDD;
  uint32_t                    duration_ms     = 1000;
  std::string                 phy_log_level   = "warning";
  std::string                 stack_log_level = "warning";

  // RF parameters
  std::string rf_device_name    = "auto";
  std::string rf_device_args    = "auto";
  std::string rf_log_level      = "info";
  float       rf_rx_gain_dB     = 20.0f;
  float       rf_freq_offset_Hz = 0.0f;

  void set_ssb_from_band(srsran_subcarrier_spacing_t scs_input)
  {
    srsran::srsran_band_helper bands;

    // Deduce band number
    uint16_t band = bands.get_band_from_dl_freq_Hz(base_carrier.dl_center_frequency_hz);

    srsran_assert(band != UINT16_MAX, "Invalid band");
    
    // Deduce point A in Hz
    double pointA_Hz =
        bands.get_abs_freq_point_a_from_center_freq(base_carrier.nof_prb, base_carrier.dl_center_frequency_hz);

    // Deduce DL center frequency ARFCN
    uint32_t pointA_arfcn = bands.freq_to_nr_arfcn(pointA_Hz);
    srsran_assert(pointA_arfcn != 0, "Invalid frequency");

    // Select a valid SSB subcarrier spacing
    ssb_scs = scs_input;
    // ssb_scs = srsran_subcarrier_spacing_30kHz;

    // Deduce SSB center frequency ARFCN
    // uint32_t ssb_arfcn = bands.get_abs_freq_ssb_arfcn(band, ssb_scs, pointA_arfcn);
    // srsran_assert(ssb_arfcn, "Invalid SSB center frequency");

    duplex_mode                     = bands.get_duplex_mode(band);
    ssb_pattern = bands.get_ssb_pattern(band, ssb_scs);
    
    // ssb_pattern                     = bands.get_ssb_pattern(band, ssb_scs);
    // base_carrier.ssb_center_freq_hz = bands.nr_arfcn_to_freq(ssb_arfcn);
  }
};

struct cell_search_result_t {
  bool                        found           = false;
  double                      ssb_abs_freq_hz = 0.0f;
  srsran_subcarrier_spacing_t ssb_scs         = srsran_subcarrier_spacing_15kHz;
  srsran_ssb_pattern_t        ssb_pattern     = SRSRAN_SSB_PATTERN_A;
  srsran_duplex_mode_t        duplex_mode     = SRSRAN_DUPLEX_MODE_FDD;
  srsran_mib_nr_t             mib             = {};
  uint32_t                    pci             = 0;
  uint32_t                    k_ssb           = 0;
  double                      abs_ssb_scs     = 0.0;
  double                      abs_pdcch_scs   = 0.0;
  int                         u = (int) ssb_scs;
  };

struct coreset0_args{
  uint32_t                    offset_rb       = 0; // CORESET offset rb
  double                      coreset0_lower_freq_hz = 0.0;
  double                      coreset0_center_freq_hz = 0.0;
  int                         n_0 = 0;
  int                         sfn_c = 0;
};

class Radio{
  public:
    int rf_index;
    srsran::rf_args_t                             rf_args;
    std::shared_ptr<srsran::radio>                r;
    std::shared_ptr<srsran::radio_interface_phy>  radio;

    srslog::basic_logger&                         logger;

    srsran::rf_buffer_t                           rf_buffer_t;
    cf_t*                                         rx_buffer;
    cf_t*                                         rx_uplink_buffer;
    uint32_t                                      slot_sz;
    srsran::rf_timestamp_t                        last_rx_time;

    cell_searcher_args_t                          args_t;
    srsue::phy_nr_sa::cell_search_args_t          cs_args;
    srsran_subcarrier_spacing_t                   ssb_scs;
    srsran::srsran_band_helper                    bands;
    srsran_ue_dl_nr_sratescs_info                 arg_scs;

    srsue::nr::cell_search                        srsran_searcher; // from cell_search.cc
    srsue::nr::cell_search::cfg_t                 srsran_searcher_cfg_t;
    srsue::nr::cell_search::args_t                srsran_searcher_args_t;
    srsue::nr::cell_search::ret_t                 cs_ret;
    uint32_t                                      nof_trials;
    cell_search_result_t                          cell;

    coreset0_args                                 coreset0_args_t;
    srsran_coreset_t                              coreset0_t;
    srsran_search_space_t*                        search_space;
    srsran_ue_sync_nr_args_t                      ue_sync_nr_args;
    srsran_ue_sync_nr_cfg_t                       sync_cfg;

    srsue::nr::slot_sync                          slot_synchronizer;
    srsran_ue_sync_nr_t                           ue_sync_nr;
    srsran_ue_sync_nr_outcome_t                   outcome;
    srsran_softbuffer_rx_t                        softbuffer;
    uint8_t*                                      data_pdcch;
    
    double pointA;
    srsran_dci_dl_nr_t dci_1_0_coreset0;
    srsran_pdcch_cfg_nr_t  pdcch_cfg;   // pdcch config for cell search and RACH
    srsran_sch_hl_cfg_nr_t pdsch_hl_cfg;
    srsran_dci_cfg_nr_t dci_cfg;
    srsran_ue_dl_nr_t ue_dl;
    srsran_ue_dl_nr_args_t ue_dl_args;
    srsran_ssb_cfg_t ssb_cfg;
    srsran_sch_cfg_nr_t pdsch_cfg;  

    asn1::rrc_nr::sib1_s sib1;

    RachUplink rach_uplink; // processing for uplink in rach
    std::vector<uint16_t> ra_rnti;
    srsran_search_space_t* ra_search_space;
    srsran::mac_rar_pdu_nr rar_pdu; // rar pdu
    uint16_t tc_rnti;
    asn1::rrc_nr::rrc_setup_s rrc_setup;
    asn1::rrc_nr::cell_group_cfg_s master_cell_group;
    uint16_t c_rnti;

    // srsran_search_space_t* tc_search_space; // used with TC-RNTI
    srsran_pdcch_cfg_nr_t  pdcch_cfg_data; // pdcch config for data communication

    Radio();  //constructor
    ~Radio(); //deconstructor

    int RadioThread();
    int RadioInitandStart();
    int DecodeMIB();
    int SyncandDownlinkInit();
    int SIB1Loop(); // downlink channel
    int MSG1Loop(); // uplink preamble
    int MSG2Loop(); // downlink RAR
    int MSG3Loop(); // uplink RRCSetupRequest
    int MSG4Loop(); // downlink RRCSetup
    
    int DCILoop();
    // int RadioStop();
};


#endif