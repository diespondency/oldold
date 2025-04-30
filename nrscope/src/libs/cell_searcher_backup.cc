#include "nrscope/hdr/cell_searcher_backup.h"
#include "nrscope/hdr/nrscope_def.h"

#include "srsran/common/band_helper.h"
#include "srsran/phy/common/phy_common_nr.h"


// shorten boost program options namespace
namespace bpo = boost::program_options;

CellSearcher::CellSearcher() : 
  logger(srslog::fetch_basic_logger("PHY")), 
  srsran_searcher(logger),
  rf_buffer_t(1),
  slot_synchronizer(logger)
  // phy("PHY")
{
  r = std::make_shared<srsran::radio>();
  radio = nullptr;
  // searcher = srsue::nr::cell_search(logger);
  srsran_searcher_args_t.max_srate_hz = 23.04e6;
  srsran_searcher_args_t.ssb_min_scs = srsran_subcarrier_spacing_15kHz;
  srsran_searcher.init(srsran_searcher_args_t);

  nof_trials = 100;

  ue_sync_nr = {};
  softbuffer = {};
  data_pdcch = NULL;
}

CellSearcher::~CellSearcher(){

}

// TODO: Maybe set the radio from config files in the future
int CellSearcher::parse_args(int argc, char** argv)
{
  int ret = SRSRAN_SUCCESS;

  bpo::options_description options("General options");
  bpo::options_description phy("Physical layer options");
  bpo::options_description stack("Stack options");
  bpo::options_description over_the_air("Mode 1: Over the air options (Default)");

  // clang-format off
 over_the_air.add_options()
     ("rf.device_name", bpo::value<std::string>(&args_t.rf_device_name)->default_value(args_t.rf_device_name), "RF Device Name")
     ("rf.device_args", bpo::value<std::string>(&args_t.rf_device_args)->default_value(args_t.rf_device_args), "RF Device arguments")
     ("rf.log_level",   bpo::value<std::string>(&args_t.rf_log_level)->default_value(args_t.rf_log_level),     "RF Log level (none, warning, info, debug)")
     ("rf.rx_gain",     bpo::value<float>(&args_t.rf_rx_gain_dB)->default_value(args_t.rf_rx_gain_dB),                           "RF Receiver gain in dB")
     ("rf.freq_offset",     bpo::value<float>(&args_t.rf_freq_offset_Hz)->default_value(args_t.rf_freq_offset_Hz),                           "RF Frequency offset")
     ;

 phy.add_options()
     ("phy.srate", bpo::value<double>(&args_t.srate_hz)->default_value(args_t.srate_hz), "Sampling Rate in Hz")
     ("phy.log.level", bpo::value<std::string>(&args_t.phy_log_level)->default_value(args_t.phy_log_level), "Physical layer logging level")
     ;

 stack.add_options()
     ("stack.log.level", bpo::value<std::string>(&args_t.stack_log_level)->default_value(args_t.stack_log_level), "Stack logging level")
     ;

 options.add(over_the_air).add(phy).add(stack).add_options()
     ("help,h",        "Show this message")
     ("duration",      bpo::value<uint32_t>(&args_t.duration_ms)->default_value(args_t.duration_ms),     "Duration of the test in milli-seconds")
     ("freq_dl", bpo::value<double>(&args_t.base_carrier.dl_center_frequency_hz)->default_value(args_t.base_carrier.dl_center_frequency_hz), "Carrier center frequency in Hz")
     ("nof_prb", bpo::value<uint32_t>(&args_t.base_carrier.nof_prb)->default_value(args_t.base_carrier.nof_prb), "Number of prbs for the cell")
     ("freq_ssb", bpo::value<double>(&args_t.base_carrier.ssb_center_freq_hz)->default_value(args_t.base_carrier.ssb_center_freq_hz), "SSB center frequency in Hz")
     ;
  // clang-format on

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).run(), vm);
    bpo::notify(vm);
  } catch (bpo::error& e) {
    std::cerr << e.what() << std::endl;
    ret = SRSRAN_ERROR;
  }

  // help option was given or error - print usage and exit
  if (vm.count("help") || ret) {
    std::cout << "Usage: " << argv[0] << " [OPTIONS] config_file" << std::endl << std::endl;
    std::cout << options << std::endl << std::endl;
    ret = SRSRAN_ERROR;
  }

  args_t.set_ssb_from_band();

  return ret;
}

int CellSearcher::init_args(){
  rf_args.type              = "multi";
  rf_args.log_level         = args_t.rf_log_level;
  rf_args.srate_hz          = args_t.srate_hz;
  rf_args.rx_gain           = args_t.rf_rx_gain_dB;
  rf_args.nof_carriers      = 1;
  rf_args.nof_antennas      = 1;
  rf_args.device_args       = args_t.rf_device_args;
  rf_args.device_name       = args_t.rf_device_name;
  rf_args.freq_offset       = args_t.rf_freq_offset_Hz;

  srsran_assert(r->init(rf_args, nullptr) == SRSRAN_SUCCESS, "Failed Radio initialisation");
  radio = std::move(r);

  // Set sampling rate
  radio->set_rx_srate(args_t.srate_hz);
  // Set DL center frequency
  radio->set_rx_freq(0, args_t.base_carrier.dl_center_frequency_hz);
  // Set Rx gain
  radio->set_rx_gain(args_t.rf_rx_gain_dB);

  return NR_SUCCESS;
}

static int slot_sync_recv_callback(void* ptr, cf_t** buffer, uint32_t nsamples, srsran_timestamp_t* ts)
{
  if (ptr == nullptr) {
    return SRSRAN_ERROR_INVALID_INPUTS;
  }
  srsran::radio* radio = (srsran::radio*)ptr;

  cf_t* buffer_ptr[SRSRAN_MAX_CHANNELS] = {};
  buffer_ptr[0]                         = buffer[0];

  srsran::rf_buffer_t rf_buffer(buffer_ptr, nsamples);

  srsran::rf_timestamp_t a;
  srsran::rf_timestamp_t &rf_timestamp = a;

  return radio->rx_now(rf_buffer, rf_timestamp);
}


int CellSearcher::start_search()
  {

  // in one band and one frequency, for each possible scs we search
  // TODO: search cells in all possible frequency bands

  for(uint32_t i = 0; i < args_t.ssb_scs.size(); i++){
  //   if(i>0 && args_t.ssb_scs[i] != args_t.ssb_scs[i-1]){
      
  //   }
    if(args_t.ssb_scs[i] == srsran_subcarrier_spacing_15kHz){
      continue;
    }

    // Allocate receive buffer
    slot_sz = (uint32_t)(args_t.srate_hz / 1000.0f / SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs[i]));
    rx_buffer = srsran_vec_cf_malloc(SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs[i]) * slot_sz);
    srsran_vec_zero(rx_buffer, SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs[i]) * slot_sz);
    // rf_buffer_t = srsran::rf_buffer_t(rx_buffer, SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs[i]) * slot_sz);

    cs_args.center_freq_hz                       = args_t.base_carrier.dl_center_frequency_hz;
    cs_args.ssb_scs                              = args_t.ssb_scs[i];
    cs_args.ssb_pattern                          = args_t.ssb_pattern[i];
    cs_args.duplex_mode                          = args_t.duplex_mode;

    uint32_t band = bands.get_band_from_dl_freq_Hz(args_t.base_carrier.dl_center_frequency_hz);
    double   ssb_bw_hz              = SRSRAN_SSB_BW_SUBC * cs_args.ssb_scs;
    double   ssb_center_freq_min_hz = args_t.base_carrier.dl_center_frequency_hz - (args_t.srate_hz * 0.7 - ssb_bw_hz) / 2.0;
    double   ssb_center_freq_max_hz = args_t.base_carrier.dl_center_frequency_hz + (args_t.srate_hz * 0.7 - ssb_bw_hz) / 2.0;
    uint32_t ssb_scs_hz             = SRSRAN_SUBC_SPACING_NR(cs_args.ssb_scs);

    //Also configure the base_carrier for later processing
    args_t.base_carrier.scs = args_t.ssb_scs[i];
    if(args_t.duplex_mode == SRSRAN_DUPLEX_MODE_TDD){
      args_t.base_carrier.ul_center_frequency_hz = args_t.base_carrier.dl_center_frequency_hz;
    }

    srsran::srsran_band_helper::sync_raster_t ss = bands.get_sync_raster(band, cs_args.ssb_scs);
    srsran_assert(ss.valid(), "Invalid synchronization raster");

    while (not ss.end()) {
      // Get SSB center frequency
      cs_args.ssb_freq_hz = ss.get_frequency();
      
      // Advance SSB frequency raster
      ss.next();

      // Calculate frequency offset between the base-band center frequency and the SSB absolute frequency
      uint32_t offset_hz = (uint32_t)std::abs(std::round(cs_args.ssb_freq_hz - args_t.base_carrier.dl_center_frequency_hz));

      // The SSB absolute frequency is invalid if it is outside the range and the offset is NOT multiple of the subcarrier
      // spacing
      if ((cs_args.ssb_freq_hz < ssb_center_freq_min_hz) or (cs_args.ssb_freq_hz > ssb_center_freq_max_hz) or
          (offset_hz % ssb_scs_hz != 0)) {
        // Skip this frequency
        continue;
      }

      srsran_searcher_cfg_t.srate_hz               = args_t.srate_hz;
      srsran_searcher_cfg_t.center_freq_hz         = cs_args.ssb_freq_hz; //args_t.base_carrier.dl_center_frequency_hz;
      srsran_searcher_cfg_t.ssb_freq_hz            = cs_args.ssb_freq_hz;
      srsran_searcher_cfg_t.ssb_scs                = args_t.ssb_scs[i];
      srsran_searcher_cfg_t.ssb_pattern            = args_t.ssb_pattern[i];
      srsran_searcher_cfg_t.duplex_mode            = args_t.duplex_mode;
      if (not srsran_searcher.start(srsran_searcher_cfg_t)) {
        std::cout << "Searcher: failed to start cell search" << std::endl;
        return NR_FAILURE;
      }
      // Set the searching frequency to ssb_freq
      // Because the srsRAN implementation use the center_freq_hz for cell searching
      cs_args.center_freq_hz = cs_args.ssb_freq_hz;
      std::cout << cs_args.ssb_freq_hz << std::endl;
      args_t.base_carrier.ssb_center_freq_hz = cs_args.ssb_freq_hz;

      radio->set_rx_freq(0, srsran_searcher_cfg_t.ssb_freq_hz);

      srsran::rf_buffer_t rf_buffer = {};
      rf_buffer.set_nof_samples(slot_sz);
      rf_buffer.set(0, rx_buffer + slot_sz);

      for(uint32_t trial=0; trial < nof_trials; trial++){
        if (trial == 0) {
          srsran_vec_cf_zero(rx_buffer, slot_sz);
        }
        srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_sz, slot_sz);

        srsran::rf_timestamp_t& rf_timestamp = last_rx_time;

        if (not radio->rx_now(rf_buffer, rf_timestamp)) {
          return SRSRAN_ERROR;
        }
        *(last_rx_time.get_ptr(0)) = rf_timestamp.get(0);

        cs_ret = srsran_searcher.run_slot(rx_buffer, slot_sz);
        if(cs_ret.result == srsue::nr::cell_search::ret_t::CELL_FOUND ){
          std::cout << "Cell Found!" << std::endl;
          std::cout << "N_id: " << cs_ret.ssb_res.N_id << std::endl;
          break;
        }
      }
    
      if(cs_ret.result == srsue::nr::cell_search::ret_t::CELL_FOUND){
        args_t.base_carrier.pci = cs_ret.ssb_res.N_id;

        // should be handed over to cell worker now, but we do this here for prototyping.
        srsran_mib_nr_t mib = {};
        if(srsran_pbch_msg_nr_mib_unpack(&cs_ret.ssb_res.pbch_msg, &mib) < SRSRAN_SUCCESS){
          ERROR("Error decoding MIB");
          return SRSRAN_ERROR;

        }
        for (int i =0; i<SRSRAN_PBCH_MSG_NR_MAX_SZ; i++){
          printf("%hhu ", cs_ret.ssb_res.pbch_msg.payload[i]);
        }
        printf("\n");

        int k_ssb = ((int)cs_ret.ssb_res.pbch_msg.k_ssb_msb)*8 + mib.ssb_offset;
        rf_buffer_t = srsran::rf_buffer_t(rx_buffer, SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs[i]) * slot_sz);

        // find coreset0 and ssb offset
        // srsran_coreset0_ssb_offset returns the offset_rb relative to ssb
        int offset_rb = srsran_coreset0_ssb_offset(mib.coreset0_idx, 
          args_t.ssb_scs[i], mib.scs_common);
        std::cout << "Coreset offset in rbs related to SSB: " << offset_rb << std::endl;

        srsran_coreset_t coreset0_t = {};
        // srsran_coreset_zero returns the offset_rb relative to pointA
        if(srsran_coreset_zero(cs_ret.ssb_res.N_id, 
                                k_ssb * SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz), 
                                args_t.ssb_scs[i], 
                                mib.scs_common, 
                                mib.coreset0_idx, 
                                &coreset0_t) == SRSRAN_SUCCESS){
          char freq_res_str[SRSRAN_CORESET_FREQ_DOMAIN_RES_SIZE] = {};

          char coreset_info[512] = {};
          srsran_coreset_to_str(&coreset0_t, coreset_info, sizeof(coreset_info));
          printf("Coreset parameter: %s", coreset_info);
        }
        // To find the position of coreset0, we need to use the offset between SSB and CORESET0,
        // because we don't know the ssb_pointA_freq_offset_Hz yet required by the srsran_coreset_zero function.
        // coreset0_t low bound freq = ssb center freq - 120 * scs (half of sc in ssb) - 
        // ssb_subcarrierOffset(from MIB) * scs - entry->offset_rb * 12(sc in one rb) * scs
        // std::cout << "ssb abs freq hz: " << found_cell.ssb_abs_freq_hz << std::endl;
        double abs_ssb_scs = SRSRAN_SUBC_SPACING_NR(args_t.ssb_scs[i]);
        double abs_pdcch_scs = SRSRAN_SUBC_SPACING_NR(mib.scs_common);
        
        // printf("cs_ret.ssb_res.pbch_msg.k_ssb_msb: %u\n", (int)cs_ret.ssb_res.pbch_msg.k_ssb_msb); 

        srsran::srsran_band_helper bands;
        double coreset0_lower_freq_hz = srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
          abs_ssb_scs - offset_rb * 12 * abs_pdcch_scs - k_ssb * SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz);
        double coreset0_center_freq_hz = coreset0_lower_freq_hz + srsran_coreset_get_bw(&coreset0_t) / 2 * 
          abs_pdcch_scs * 12;
        // double coreset0_center_freq_hz = coreset0_lower_freq_hz + args_t.base_carrier.nof_prb * 12 / 2 * 
        //   abs_pdcch_scs;

        std::cout << "k_ssb: " << k_ssb << std::endl;
        std::cout << "lower ssb freq hz: " << srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
          abs_ssb_scs << std::endl;
        std::cout << "coreset0_lower freq hz: " << coreset0_lower_freq_hz << std::endl;
        std::cout << "coreset0 center freq hz: " << coreset0_center_freq_hz << std::endl;
        std::cout << "ssb_lower_freq hz: " << srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
          abs_ssb_scs << std::endl;
        std::cout << "coreset0_bw: " << srsran_coreset_get_bw(&coreset0_t) << std::endl;
        // the total used prb for coreset0 is 48 -> 17.28 MHz bw
        
        std::cout << "mib pdcch-configSIB1.coreset0_idx: " << mib.coreset0_idx << std::endl;
        std::cout << "mib pdcch-configSIB1.searchSpaceZero: " << mib.ss0_idx << std::endl;
        printf("mib ssb-index: %u\n", mib.ssb_idx);


        // we should implement the table checking later
        coreset_zero_t_f_entry_nrscope coreset_zero_cfg;
        // get coreset_zero's position in time and frequency domain
        if(coreset_zero_t_f_nrscope(mib.ss0_idx, mib.ssb_idx, coreset0_t.duration, &coreset_zero_cfg) < 
          SRSRAN_SUCCESS){
          ERROR("Error checking table 13-11");
          return SRSRAN_ERROR;
        }

        int u = (int)args_t.ssb_scs[i]; // scs = 30kHz, which is u=1
        int n_0 = (coreset_zero_cfg.O * (int)pow(2, u) + (int)floor(mib.ssb_idx * coreset_zero_cfg.M)) % 20;
        std::cout << "slot position for coreset 0: " << n_0 << std::endl;

        // sfn_c = 0, in even system frame
        // sfn_c = 1, in odd system frame    
        int sfn_c = (int)(floor(coreset_zero_cfg.O * pow(2, u) + floor(mib.ssb_idx * coreset_zero_cfg.M)) / 20) % 2;
        std::cout << "sfn_c for coreset 0: " << sfn_c << std::endl; 

        args_t.base_carrier.nof_prb = srsran_coreset_get_bw(&coreset0_t);


        //***** DL args Config Start *****//
        // Should put this part in the decoder class
        static srsran_pdcch_cfg_nr_t  pdcch_cfg = {};
        static srsran_sch_hl_cfg_nr_t pdsch_hl_cfg = {};
        srsran_dci_cfg_nr_t dci_cfg = {};
        static srsran_ue_dl_nr_t      ue_dl = {};
        srsran_ue_dl_nr_args_t ue_dl_args        = {};

        // this part of bw is set according to the coreset0_bw
        dci_cfg.bwp_dl_initial_bw   = 60;//srsran_coreset_get_bw(&coreset0_t);
        dci_cfg.bwp_ul_initial_bw   = 60;//srsran_coreset_get_bw(&coreset0_t);
        dci_cfg.bwp_dl_active_bw    = 60;//srsran_coreset_get_bw(&coreset0_t);
        dci_cfg.bwp_ul_active_bw    = 60;//srsran_coreset_get_bw(&coreset0_t);
        dci_cfg.monitor_common_0_0  = true;
        dci_cfg.monitor_0_0_and_1_0 = true;

        ue_dl_args.nof_rx_antennas               = 1;
        ue_dl_args.pdsch.sch.disable_simd        = false;
        ue_dl_args.pdsch.sch.decoder_use_flooded = false;
        ue_dl_args.pdsch.measure_evm             = true;
        ue_dl_args.pdcch.disable_simd            = false;
        ue_dl_args.pdcch.measure_evm             = true;
        ue_dl_args.nof_max_prb                   = 60;//srsran_coreset_get_bw(&coreset0_t);

        pdcch_cfg.coreset_present[0] = true;
        // Setup PDSCH DMRS (also signaled through MIB)
        pdsch_hl_cfg.typeA_pos = mib.dmrs_typeA_pos;
        // set coreset0 bandwidth
        dci_cfg.coreset0_bw = srsran_coreset_get_bw(&coreset0_t);

        srsran_search_space_t* search_space = &pdcch_cfg.search_space[0];
        pdcch_cfg.search_space_present[0]   = true;
        search_space->id                    = 0;
        search_space->coreset_id            = 0;
        search_space->type                  = srsran_search_space_type_common_0;
        // search_space->formats[0]            = srsran_dci_format_nr_0_0;
        search_space->formats[0]            = srsran_dci_format_nr_1_0;
        search_space->nof_formats           = 1;
        for (uint32_t L = 0; L < SRSRAN_SEARCH_SPACE_NOF_AGGREGATION_LEVELS_NR; L++) {
          search_space->nof_candidates[L] = srsran_pdcch_nr_max_candidates_coreset(&coreset0_t, L);
        }

        //  Configure RA search space
        pdcch_cfg.ra_search_space_present = false;
        pdcch_cfg.ra_search_space         = *search_space;
        pdcch_cfg.ra_search_space.type    = srsran_search_space_type_common_1;
        pdcch_cfg.coreset[0] = coreset0_t;  

        // it appears the srsRAN is build on 15kHz scs, we need to use the srate and 
        // scs to calculate the correct subframe size 
        srsran_ue_dl_nr_sratescs_info arg_scs;
        arg_scs.srate = args_t.srate_hz;
        arg_scs.scs = mib.scs_common;
        arg_scs.coreset_offset_scs = (cs_args.ssb_freq_hz - coreset0_center_freq_hz) / abs_pdcch_scs;
        arg_scs.coreset_slot = (uint32_t)n_0;

        std::cout << "cs_args.ssb_freq_hz - coreset0_center_freq_hz: " << cs_args.ssb_freq_hz - coreset0_center_freq_hz << std::endl;
        std::cout << "arg_scs.coreset_offset_scs: " << arg_scs.coreset_offset_scs << std::endl; 
        // we need to set ue_dl.sf_symbols here to set the out_buffer correctly.
        if (srsran_ue_dl_nr_init_nrscope(&ue_dl, rf_buffer_t.to_cf_t(), &ue_dl_args, arg_scs)) {
          ERROR("Error UE DL");
          return SRSRAN_ERROR;
        }
        // the fft data is fed into ue_dl.fft[0].tmp
        if (srsran_ue_dl_nr_set_carrier_nrscope(&ue_dl, &args_t.base_carrier, arg_scs)) {
          ERROR("Error setting SCH NR carrier");
          return SRSRAN_ERROR;
        }

        // config the dci length for the current size, but it seems to only config for c-rnti dci 1_0
        if (srsran_ue_dl_nr_set_pdcch_config(&ue_dl, &pdcch_cfg, &dci_cfg)) {
          ERROR("Error setting CORESET");
          return SRSRAN_ERROR;
        }

        if (srsran_softbuffer_rx_init_guru(&softbuffer, SRSRAN_SCH_NR_MAX_NOF_CB_LDPC, SRSRAN_LDPC_MAX_LEN_ENCODED_CB) <
            SRSRAN_SUCCESS) {
          ERROR("Error init soft-buffer");
          return SRSRAN_ERROR;
        }

        data_pdcch = srsran_vec_u8_malloc(SRSRAN_SLOT_MAX_NOF_BITS_NR);
        if (data_pdcch == NULL) {
          ERROR("Error malloc");
          return SRSRAN_ERROR;
        }
        //***** DL args Config End *****//

        //***** Slot Sync Start *****//
        srsran_ue_sync_nr_args_t ue_sync_nr_args = {};
        ue_sync_nr_args.max_srate_hz             = srsran_searcher_args_t.max_srate_hz;
        ue_sync_nr_args.min_scs                  = srsran_searcher_args_t.ssb_min_scs;
        ue_sync_nr_args.nof_rx_channels          = 1;
        ue_sync_nr_args.disable_cfo              = false;
        ue_sync_nr_args.pbch_dmrs_thr            = 0.5;
        ue_sync_nr_args.cfo_alpha                = 0.1;
        ue_sync_nr_args.recv_obj                 = radio.get();
        ue_sync_nr_args.recv_callback            = slot_sync_recv_callback;

        //Manually setup the CFO
        ue_sync_nr.cfo_hz = rf_args.freq_offset;

        if (srsran_ue_sync_nr_init(&ue_sync_nr, &ue_sync_nr_args) < SRSRAN_SUCCESS) {
          std::cout << "Error initiating UE SYNC NR object" << std::endl;
          logger.error("Error initiating UE SYNC NR object");
          return SRSRAN_ERROR;
        }

        srsran_ssb_cfg_t ssb_cfg = {};
        ssb_cfg.srate_hz                                  = args_t.srate_hz;
        ssb_cfg.center_freq_hz                            = args_t.base_carrier.dl_center_frequency_hz;
        ssb_cfg.ssb_freq_hz                               = cs_args.ssb_freq_hz;
        ssb_cfg.scs                                       = cs_args.ssb_scs;
        ssb_cfg.pattern                                   = cs_args.ssb_pattern;
        ssb_cfg.duplex_mode                               = cs_args.duplex_mode;
        ssb_cfg.periodicity_ms                            = 10;

        srsran_ue_sync_nr_cfg_t cfg = {};
        cfg.N_id                    = cs_ret.ssb_res.N_id;
        cfg.ssb                     = ssb_cfg;
        cfg.ssb.srate_hz            = args_t.srate_hz;
        if (srsran_ue_sync_nr_set_cfg(&ue_sync_nr, &cfg) < SRSRAN_SUCCESS) {
          printf("SYNC: failed to set cell configuration for N_id %d", cfg.N_id);
          logger.error("SYNC: failed to set cell configuration for N_id %d", cfg.N_id);
          return SRSRAN_ERROR;
        }
                          
        bool found_dci = false;
        srsran_dci_dl_nr_t dci_dl_rx = {};
        char str[1024] = {};

        std::cout << "Start syncing" << std::endl;
        // synced subframes
        for(int sync_id = 0; sync_id < 500; sync_id++){
          srsran_ue_sync_nr_outcome_t outcome = {};
          if (srsran_ue_sync_nr_zerocopy(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome) < SRSRAN_SUCCESS) {
            std::cout << "SYNC: error in zerocopy" << std::endl;
            logger.error("SYNC: error in zerocopy");
            return false;
          }

          // If in sync, update slot index
          // the synced data is stored in rf_buffer_t.to_cf_t()[0]
          if (outcome.in_sync) {

            std::cout << "System frame idx: " << outcome.sfn << std::endl;
            std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;
        
            // ***** CORESET0 Decoder *****//
            // Actual decode
            for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
              srsran_slot_cfg_t slot = {0};
              slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;

              // Move the buffer forward for one slot
              srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);

              if(!found_dci){
                // We haven't found the DCI, we need to search it
                if((sfn_c == 0 && outcome.sfn % 2 == 0) || (sfn_c == 0 && outcome.sfn % 2 == 1)) {
                  if((outcome.sf_idx) == (uint32_t)(n_0 / 2)){
                    // Check the fft plan and how does it manipulate the buffer
                    srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);
                    // Blind search
                    int nof_found_dci = srsran_ue_dl_nr_find_dl_dci(&ue_dl, &slot, 0xFFFF, srsran_rnti_type_si, &dci_dl_rx, 1);
                    if (nof_found_dci < SRSRAN_SUCCESS) {
                      ERROR("Error in blind search");
                      return SRSRAN_ERROR;
                    }

                    // Print PDCCH blind search candidates
                    for (uint32_t pdcch_idx = 0; pdcch_idx < ue_dl.pdcch_info_count; pdcch_idx++) {
                      const srsran_ue_dl_nr_pdcch_info_t* info = &(ue_dl.pdcch_info[pdcch_idx]);
                      printf("PDCCH: %s-rnti=0x%x, crst_id=%d, ss_type=%s, ncce=%d, al=%d, EPRE=%+.2f, RSRP=%+.2f, corr=%.3f; "
                        "nof_bits=%d; crc=%s;\n",
                        srsran_rnti_type_str_short(info->dci_ctx.rnti_type),
                        info->dci_ctx.rnti,
                        info->dci_ctx.coreset_id,
                        srsran_ss_type_str(info->dci_ctx.ss_type),
                        info->dci_ctx.location.ncce,
                        info->dci_ctx.location.L,
                        info->measure.epre_dBfs,
                        info->measure.rsrp_dBfs,
                        info->measure.norm_corr,
                        info->nof_bits,
                        info->result.crc ? "OK" : "KO");
                    }

                    if (nof_found_dci < 1) {
                      printf("No DCI found :'(\n");
                      continue;
                    }

                    srsran_dci_dl_nr_to_str(&(ue_dl.dci), &dci_dl_rx, str, (uint32_t)sizeof(str));
                    printf("Found DCI: %s\n", str);
                    found_dci = true;
                  }
                }
              }else{
                // We have found the DCI and decoded it. Then we decode the PDSCH.
                // demodulate the OFDM symbols with different bandwidth
                // for (uint32_t ant_idx = 0; ant_idx < ue_dl.nof_rx_antennas; ant_idx++) {
                //   srsran_ofdm_rx_sf_nrscope(&(ue_dl.fft[ant_idx]), (int)arg_scs.scs, arg_scs.coreset_offset_scs);
                // }
                srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);
                // Convert DCI to PDSCH transmission
                srsran_sch_cfg_nr_t pdsch_cfg = {};
                // if (rnti_type == srsran_rnti_type_ra) {
                //   pdsch_hl_cfg.common_time_ra[0].k            = 0;
                //   pdsch_hl_cfg.common_time_ra[0].mapping_type = srsran_sch_mapping_type_A;
                //   pdsch_hl_cfg.common_time_ra[0].sliv =
                //       srsran_ra_type2_to_riv(SRSRAN_NSYMB_PER_SLOT_NR - 1, 1, SRSRAN_NSYMB_PER_SLOT_NR);
                //   pdsch_hl_cfg.nof_common_time_ra = 1;
                // }
                if (srsran_ra_dl_dci_to_grant_nr(&(args_t.base_carrier), &slot, &pdsch_hl_cfg, &dci_dl_rx, &pdsch_cfg, &pdsch_cfg.grant) <
                    SRSRAN_SUCCESS) {
                  ERROR("Error decoding PDSCH search");
                  return SRSRAN_ERROR;
                }

                srsran_sch_cfg_nr_info(&pdsch_cfg, str, (uint32_t)sizeof(str));
                // printf("PDSCH: %s\n", str);

                // Set softbuffer
                pdsch_cfg.grant.tb[0].softbuffer.rx = &softbuffer;

                // Prepare PDSCH result
                srsran_pdsch_res_nr_t pdsch_res = {};
                pdsch_res.tb[0].payload         = data_pdcch;

                // Decode PDSCH
                if (srsran_ue_dl_nr_decode_pdsch(&ue_dl, &slot, &pdsch_cfg, &pdsch_res) < SRSRAN_SUCCESS) {
                  printf("Error decoding PDSCH search\n");
                  continue;
                  // return SRSRAN_ERROR;
                }

                if (!pdsch_res.tb[0].crc) {
                  printf("Error decoding PDSCH\n");
                  continue;
                  // return SRSRAN_ERROR;
                }

                printf("Decoded PDSCH (%d B)\n", pdsch_cfg.grant.tb[0].tbs / 8);
                srsran_vec_fprint_byte(stdout, pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8);

                // check payload is not all null
                bool all_zero = true;
                for (int i = 0; i < pdsch_cfg.grant.tb[0].tbs / 8; ++i) {
                  if (pdsch_res.tb[0].payload[i] != 0x0) {
                    all_zero = false;
                    break;
                  }
                }
                if (all_zero) {
                  ERROR("PDSCH payload is all zeros");
                  // return SRSRAN_ERROR;
                }
              }
            }
          }
        }
      }
    }
  }
  return NR_SUCCESS;
}

// int main(int argc, char** argv)
// {
  // srsran_debug_handle_crash(argc, argv);

  // // Initialise logging infrastructure
  // srslog::init();

  // CellSearcher *cell_searcher = new CellSearcher();

  // // Parse Test arguments
  // srsran_assert(cell_searcher->parse_args(argc, argv) == SRSRAN_SUCCESS, "Failed to parse arguments");
  // cell_searcher->init_args();

  // std::cout << "Parameters and radio set." << std::endl;
  // std::cout << "cell_searcher args_t: " << 
  //   cell_searcher->args_t.base_carrier.dl_center_frequency_hz << std::endl;
  
  // cell_searcher->start_search();

  // return SRSRAN_SUCCESS;
// }
