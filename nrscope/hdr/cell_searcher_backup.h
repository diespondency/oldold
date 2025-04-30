#ifndef CELL_SEARCHER_H
#define CELL_SEARCHER_H

#include "srsran/common/band_helper.h"
#include "srsran/common/crash_handler.h"
#include "srsran/common/string_helpers.h"
#include "srsue/hdr/phy/phy_nr_sa.h"
#include "srsue/hdr/phy/nr/cell_search.h"
#include "test/phy/dummy_ue_stack.h"
#include "srsue/hdr/stack/ue_stack_nr.h"
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>

#include "srsran/srslog/logger.h"
#include <iostream>
#include <cmath>

struct cell_searcher_args_t {
  // Generic parameters
  double                      srate_hz        = 11.52e6;
  srsran_carrier_nr_t         base_carrier    = SRSRAN_DEFAULT_CARRIER_NR;
  std::vector<srsran_ssb_pattern_t>        ssb_pattern;
  std::vector<srsran_subcarrier_spacing_t> ssb_scs;
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

  // TODO: return and search all possible ssb in FR1
  void set_ssb_from_band()
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
    ssb_scs = bands.get_all_ssb_scs(band);
    // ssb_scs = srsran_subcarrier_spacing_30kHz;

    // Deduce SSB center frequency ARFCN
    // uint32_t ssb_arfcn = bands.get_abs_freq_ssb_arfcn(band, ssb_scs, pointA_arfcn);
    // srsran_assert(ssb_arfcn, "Invalid SSB center frequency");

    duplex_mode                     = bands.get_duplex_mode(band);
    for(srsran_subcarrier_spacing_t scs_ele : ssb_scs){
      ssb_pattern.push_back(bands.get_ssb_pattern(band, scs_ele));
    }
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
};


class CellSearcher{
  public:
    cell_searcher_args_t                          args_t;
    srsran::rf_args_t                             rf_args;
    std::shared_ptr<srsran::radio>                r;
    std::shared_ptr<srsran::radio_interface_phy>  radio;
    cell_search_result_t                          result_t;
    // std::vector<cell_search_result_t>             result_t;

    srsran::srsran_band_helper                    bands;

    // cell_search args
    srslog::basic_logger&                         logger;
    srsue::nr::cell_search                        srsran_searcher; // from cell_search.cc
    srsue::nr::cell_search::cfg_t                 srsran_searcher_cfg_t;
    srsue::nr::cell_search::args_t                srsran_searcher_args_t;
    srsue::nr::cell_search::ret_t                 cs_ret;

    srsue::phy_nr_sa::cell_search_args_t          cs_args;

    srsran::rf_buffer_t                           rf_buffer_t;
    cf_t*                                         rx_buffer;
    uint32_t                                      slot_sz;
    srsran::rf_timestamp_t                        last_rx_time;

    uint32_t                                      nof_trials;

    srsue::nr::slot_sync                          slot_synchronizer;
    srsran_ue_sync_nr_t                           ue_sync_nr;
    srsran_softbuffer_rx_t                        softbuffer;
    uint8_t*                                      data_pdcch;

    // srsue::phy_nr_sa                              phy;
    // ue_dummy_stack                                stack; // for initialization of slot sync
    
    CellSearcher();
    ~CellSearcher();

    int parse_args(int argc, char** argv);
    int init_args();
    int start_search();
    
};

#endif