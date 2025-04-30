#include "nrscope/hdr/radio_nr.h"
#include "nrscope/hdr/nrscope_def.h"

#include "srsran/common/band_helper.h"
#include "srsran/phy/common/phy_common_nr.h"
#include "nrscope/hdr/asn_decoder.h"

std::mutex lock_radio_nr;

Radio::Radio() : 
  logger(srslog::fetch_basic_logger("PHY")), 
  srsran_searcher(logger),
  rf_buffer_t(1),
  slot_synchronizer(logger),
  rach_uplink()
{
  r = std::make_shared<srsran::radio>();
  radio = nullptr;

  nof_trials = 100;
  srsran_searcher_args_t.max_srate_hz = 30.72e6;
  srsran_searcher_args_t.ssb_min_scs = srsran_subcarrier_spacing_15kHz;
  srsran_searcher.init(srsran_searcher_args_t);

  pdcch_cfg = {};
  pdsch_hl_cfg = {};
  dci_cfg = {};
  ue_dl = {};
  ue_dl_args = {};
  ssb_cfg = {};

  ue_sync_nr = {};
  softbuffer = {};
  outcome = {};
  ue_sync_nr_args = {};
  sync_cfg = {};
  data_pdcch = NULL;

}

Radio::~Radio() {
}

int Radio::RadioThread(){
  RadioInitandStart();
  return NR_SUCCESS;
}

int Radio::RadioInitandStart(){

  srsran_assert(r->init(rf_args, nullptr) == SRSRAN_SUCCESS, "Failed Radio initialisation");
  radio = std::move(r);

  // Cell Searcher parameters  
  args_t.srate_hz = rf_args.srate_hz;
  rf_args.dl_freq = args_t.base_carrier.dl_center_frequency_hz;
  args_t.rf_device_name = rf_args.device_name;
  args_t.rf_device_args = rf_args.device_args;
  args_t.rf_log_level = "info";
  args_t.rf_rx_gain_dB = rf_args.rx_gain;
  args_t.rf_freq_offset_Hz = rf_args.freq_offset;
  args_t.phy_log_level   = "warning";
  args_t.stack_log_level = "warning";
  args_t.duration_ms = 1000;

  // Set sampling rate
  radio->set_rx_srate(rf_args.srate_hz);
  // Set DL center frequency
  radio->set_rx_freq(0, (double)rf_args.dl_freq);
  // Set Rx gain
  radio->set_rx_gain(rf_args.rx_gain);
  
  args_t.set_ssb_from_band(ssb_scs);
  args_t.base_carrier.scs = args_t.ssb_scs;
  if(args_t.duplex_mode == SRSRAN_DUPLEX_MODE_TDD){
    args_t.base_carrier.ul_center_frequency_hz = args_t.base_carrier.dl_center_frequency_hz;
  }

  // Allocate receive buffer
  slot_sz = (uint32_t)(rf_args.srate_hz / 1000.0f / SRSRAN_NOF_SLOTS_PER_SF_NR(ssb_scs));
  rx_buffer = srsran_vec_cf_malloc(SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs) * slot_sz);
  srsran_vec_zero(rx_buffer, SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs) * slot_sz);

  cs_args.center_freq_hz = args_t.base_carrier.dl_center_frequency_hz;
  cs_args.ssb_freq_hz = args_t.base_carrier.dl_center_frequency_hz;
  cs_args.ssb_scs = args_t.ssb_scs;
  cs_args.ssb_pattern = args_t.ssb_pattern;
  cs_args.duplex_mode = args_t.duplex_mode;

  uint32_t band = bands.get_band_from_dl_freq_Hz(args_t.base_carrier.dl_center_frequency_hz);
  double ssb_bw_hz = SRSRAN_SSB_BW_SUBC * cs_args.ssb_scs;
  double ssb_center_freq_min_hz = args_t.base_carrier.dl_center_frequency_hz - (args_t.srate_hz * 0.7 - ssb_bw_hz) / 2.0;
  double ssb_center_freq_max_hz = args_t.base_carrier.dl_center_frequency_hz + (args_t.srate_hz * 0.7 - ssb_bw_hz) / 2.0;
  uint32_t ssb_scs_hz = SRSRAN_SUBC_SPACING_NR(cs_args.ssb_scs);
  
  srsran::srsran_band_helper::sync_raster_t ss = bands.get_sync_raster(band, cs_args.ssb_scs);
  srsran_assert(ss.valid(), "Invalid synchronization raster");

  while (not ss.end()) {
    // Get SSB center frequency
    cs_args.ssb_freq_hz = ss.get_frequency();
    // Advance SSB frequency raster
    ss.next();

    // Calculate frequency offset between the base-band center frequency and the SSB absolute frequency
    uint32_t offset_hz = (uint32_t)std::abs(std::round(cs_args.ssb_freq_hz - args_t.base_carrier.dl_center_frequency_hz));

    // The SSB absolute frequency is invalid if it is outside the range and the offset is NOT multiple of the subcarrier spacing
    if ((cs_args.ssb_freq_hz < ssb_center_freq_min_hz) or (cs_args.ssb_freq_hz > ssb_center_freq_max_hz) or
        (offset_hz % ssb_scs_hz != 0)) {
      // Skip this frequency
      continue;
    }

    srsran_searcher_cfg_t.srate_hz = args_t.srate_hz;
    srsran_searcher_cfg_t.center_freq_hz = cs_args.ssb_freq_hz; //args_t.base_carrier.dl_center_frequency_hz;
    srsran_searcher_cfg_t.ssb_freq_hz = cs_args.ssb_freq_hz;
    srsran_searcher_cfg_t.ssb_scs = args_t.ssb_scs;
    srsran_searcher_cfg_t.ssb_pattern = args_t.ssb_pattern;
    srsran_searcher_cfg_t.duplex_mode = args_t.duplex_mode;
    if (not srsran_searcher.start(srsran_searcher_cfg_t)) {
      std::cout << "Searcher: failed to start cell search" << std::endl;
      return NR_FAILURE;
    }
    // Set the searching frequency to ssb_freq
    // Because the srsRAN implementation use the center_freq_hz for cell searching
    cs_args.center_freq_hz = cs_args.ssb_freq_hz;
    std::cout << cs_args.ssb_freq_hz << std::endl;
    args_t.base_carrier.ssb_center_freq_hz = cs_args.ssb_freq_hz;

    std::cout << "srsran_searcher_cfg_t.ssb_freq_hz: " << srsran_searcher_cfg_t.ssb_freq_hz << std::endl;
    radio->release_freq(0);
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
        printf("found cell in this slot\n");
        break;
      }
    }
    if(cs_ret.result == srsue::nr::cell_search::ret_t::CELL_FOUND){
      std::cout << "Cell Found!" << std::endl;
      std::cout << "N_id: " << cs_ret.ssb_res.N_id << std::endl;
      std::cout << "Decoding MIB..." << std::endl;

      // start uplink loop
      // rach_uplink.RachUpInit(SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs) * slot_sz);
      // in one thread, seems good.
      rx_uplink_buffer = srsran_vec_cf_malloc(SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs) * slot_sz);
      
      if(DecodeMIB() < NR_SUCCESS){
        ERROR("Error decoding MIB");
        return NR_FAILURE;
      }

      if(SyncandDownlinkInit() < NR_SUCCESS){
        ERROR("Error decoding MIB");
        return NR_FAILURE;
      }

      if(SIB1Loop() < NR_SUCCESS){
        ERROR("Error in SIB1Loop");
        return NR_FAILURE;
      }

      // if(MSG1Loop() < NR_SUCCESS){
      //   ERROR("Error in MSG1Loop");
      //   return NR_FAILURE;
      // }

      if(MSG2Loop() < NR_SUCCESS){
        ERROR("Error in MSG2Loop");
        return NR_FAILURE;
      }

      for (auto& subpdu : rar_pdu.get_subpdus()) {
        if(subpdu.has_rapid()){
          printf("TC-RNTI: %u\n", subpdu.get_temp_crnti());
          printf("RAPID: %u\n", subpdu.get_rapid());
        }
      }

      // if(MSG3Loop() < NR_SUCCESS){
      //   ERROR("Error in MSG3Loop");
      //   return NR_FAILURE;
      // }

      if(MSG4Loop() < NR_SUCCESS){
        ERROR("Error in MSG4Loop");
        return NR_FAILURE;
      }

      if(DCILoop() < NR_SUCCESS){
        ERROR("Error in DCILoop");
        return NR_FAILURE;
      }
    }
  }
  return NR_SUCCESS;
}

int Radio::DecodeMIB(){
  args_t.base_carrier.pci = cs_ret.ssb_res.N_id;

  if(srsran_pbch_msg_nr_mib_unpack(&cs_ret.ssb_res.pbch_msg, &cell.mib) < SRSRAN_SUCCESS){
    ERROR("Error decoding MIB");
    return SRSRAN_ERROR;
  }
  // printf("MIB payload: ");
  // for (int i =0; i<SRSRAN_PBCH_MSG_NR_MAX_SZ; i++){
  //   printf("%hhu ", cs_ret.ssb_res.pbch_msg.payload[i]);
  // }
  // printf("\n");
  std::cout << "cell.mib.ssb_offset: " << cell.mib.ssb_offset << std::endl;
  std::cout << "((int)cs_ret.ssb_res.pbch_msg.k_ssb_msb): " << ((int)cs_ret.ssb_res.pbch_msg.k_ssb_msb) << std::endl;

  cell.k_ssb = cell.mib.ssb_offset; // already added the msb of k_ssb
  rf_buffer_t = srsran::rf_buffer_t(rx_buffer, SRSRAN_NOF_SLOTS_PER_SF_NR(args_t.ssb_scs) * slot_sz);

  // srsran_coreset0_ssb_offset returns the offset_rb relative to ssb
  // nearly all bands in FR1 have min bandwidth 5 or 10 MHz, so there are only 5 entries here.  
  coreset0_args_t.offset_rb = srsran_coreset0_ssb_offset(cell.mib.coreset0_idx, 
    args_t.ssb_scs, cell.mib.scs_common);
  std::cout << "Coreset offset in rbs related to SSB: " << coreset0_args_t.offset_rb << std::endl;

  coreset0_t = {};
  // srsran_coreset_zero returns the offset_rb relative to pointA
  if(srsran_coreset_zero(cs_ret.ssb_res.N_id, 
                         0, //cell.k_ssb * SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz), 
                         args_t.ssb_scs, 
                         cell.mib.scs_common, 
                         cell.mib.coreset0_idx, 
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
  cell.abs_ssb_scs = SRSRAN_SUBC_SPACING_NR(args_t.ssb_scs);
  cell.abs_pdcch_scs = SRSRAN_SUBC_SPACING_NR(cell.mib.scs_common);
  
  srsran::srsran_band_helper bands;
  coreset0_args_t.coreset0_lower_freq_hz = srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
    cell.abs_ssb_scs - coreset0_args_t.offset_rb * NRSCOPE_NSC_PER_RB_NR * cell.abs_pdcch_scs - 
    cell.k_ssb * SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz);
  coreset0_args_t.coreset0_center_freq_hz = coreset0_args_t.coreset0_lower_freq_hz + srsran_coreset_get_bw(&coreset0_t) / 2 * 
    cell.abs_pdcch_scs * NRSCOPE_NSC_PER_RB_NR;

  std::cout << "k_ssb: " << cell.k_ssb << std::endl;
  std::cout << "ssb freq hz: " << srsran_searcher_cfg_t.ssb_freq_hz << std::endl;
  std::cout << "coreset0_lower freq hz: " << coreset0_args_t.coreset0_lower_freq_hz << std::endl;
  std::cout << "coreset0 center freq hz: " << coreset0_args_t.coreset0_center_freq_hz << std::endl;
  std::cout << "ssb_lower_freq hz: " << srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
    cell.abs_ssb_scs << std::endl;
  std::cout << "coreset0_bw: " << srsran_coreset_get_bw(&coreset0_t) << std::endl;
  std::cout << "coreset0_nof_symb: " << coreset0_t.duration << std::endl;
  // the total used prb for coreset0 is 48 -> 17.28 MHz bw
  
  std::cout << "mib pdcch-configSIB1.coreset0_idx: " << cell.mib.coreset0_idx << std::endl;
  std::cout << "mib pdcch-configSIB1.searchSpaceZero: " << cell.mib.ss0_idx << std::endl;
  printf("mib ssb-index: %u\n", cell.mib.ssb_idx);

  coreset_zero_t_f_entry_nrscope coreset_zero_cfg;
  // get coreset_zero's position in time domain
  // check table 38.213, 13-11, because USRP can only support FR1.
  if(coreset_zero_t_f_nrscope(cell.mib.ss0_idx, cell.mib.ssb_idx, coreset0_t.duration, &coreset_zero_cfg) < 
    SRSRAN_SUCCESS){
    ERROR("Error checking table 13-11");
    return SRSRAN_ERROR;
  }

  cell.u = (int)args_t.ssb_scs; 
  coreset0_args_t.n_0 = (coreset_zero_cfg.O * (int)pow(2, cell.u) + 
    (int)floor(cell.mib.ssb_idx * coreset_zero_cfg.M)) % SRSRAN_NSLOTS_X_FRAME_NR(cell.u);
  std::cout << "slot position for coreset 0: " << coreset0_args_t.n_0 << std::endl;
  // sfn_c = 0, in even system frame, sfn_c = 1, in odd system frame    
  coreset0_args_t.sfn_c = (int)(floor(coreset_zero_cfg.O * pow(2, cell.u) + 
    floor(cell.mib.ssb_idx * coreset_zero_cfg.M)) / SRSRAN_NSLOTS_X_FRAME_NR(cell.u)) % 2;
  args_t.base_carrier.nof_prb = srsran_coreset_get_bw(&coreset0_t);

  // args_t.base_carrier.dl_center_frequency_hz = (double)srsran_searcher_cfg_t.ssb_freq_hz;
  // radio->reset();
  // radio->set_rx_freq(0, (double)srsran_searcher_cfg_t.ssb_freq_hz);
  // radio->rx_now();

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
  *ts = a.get(0);

  return radio->rx_now(rf_buffer, rf_timestamp);
}

int Radio::SyncandDownlinkInit(){
  // coreset0_t.offset_rb = 1; // for debugging
  //***** DL args Config Start *****//
  // this part of bw is set according to the coreset0_bw
  dci_cfg.bwp_dl_initial_bw   = 275;//srsran_coreset_get_bw(&coreset0_t);
  dci_cfg.bwp_ul_initial_bw   = 275;//srsran_coreset_get_bw(&coreset0_t);
  dci_cfg.bwp_dl_active_bw    = 275;//srsran_coreset_get_bw(&coreset0_t);
  dci_cfg.bwp_ul_active_bw    = 275;//srsran_coreset_get_bw(&coreset0_t);
  dci_cfg.monitor_common_0_0  = true;
  dci_cfg.monitor_0_0_and_1_0 = true;
  dci_cfg.monitor_0_1_and_1_1 = true;

  ue_dl_args.nof_rx_antennas               = 1;
  ue_dl_args.pdsch.sch.disable_simd        = false;
  ue_dl_args.pdsch.sch.decoder_use_flooded = false;
  ue_dl_args.pdsch.measure_evm             = true;
  ue_dl_args.pdcch.disable_simd            = false;
  ue_dl_args.pdcch.measure_evm             = true;
  ue_dl_args.nof_max_prb                   = 275;//srsran_coreset_get_bw(&coreset0_t);

  pdcch_cfg.coreset_present[0] = true;
  // Setup PDSCH DMRS (also signaled through MIB)
  pdsch_hl_cfg.typeA_pos = cell.mib.dmrs_typeA_pos;
  // set coreset0 bandwidth
  dci_cfg.coreset0_bw = srsran_coreset_get_bw(&coreset0_t);
  
  search_space = &pdcch_cfg.search_space[0];
  pdcch_cfg.search_space_present[0]   = true;
  search_space->id                    = 0;
  search_space->coreset_id            = 0;
  search_space->type                  = srsran_search_space_type_common_0;
  search_space->formats[0]            = srsran_dci_format_nr_1_0;
  search_space->nof_formats           = 1;
  for (uint32_t L = 0; L < SRSRAN_SEARCH_SPACE_NOF_AGGREGATION_LEVELS_NR; L++) {
    search_space->nof_candidates[L] = srsran_pdcch_nr_max_candidates_coreset(&coreset0_t, L);
  }
  pdcch_cfg.coreset[0] = coreset0_t; 

  // it appears the srsRAN is build on 15kHz scs, we need to use the srate and 
  // scs to calculate the correct subframe size 
  arg_scs.srate = args_t.srate_hz;
  arg_scs.scs = cell.mib.scs_common;
  // radio->release_freq(0);
  // radio->set_rx_freq(0, coreset0_args_t.coreset0_center_freq_hz);
  // arg_scs.coreset_offset_scs = 0;
  arg_scs.coreset_offset_scs = (cs_args.ssb_freq_hz - coreset0_args_t.coreset0_center_freq_hz) / cell.abs_pdcch_scs;// + 12;
  arg_scs.coreset_slot = (uint32_t)coreset0_args_t.n_0;
  std::cout << "arg_scs.coreset_offset_scs: " << arg_scs.coreset_offset_scs << std::endl; 

  // we need to set ue_dl.sf_symbols here to set the out_buffer correctly.
  if (srsran_ue_dl_nr_init_nrscope(&ue_dl, rf_buffer_t.to_cf_t(), &ue_dl_args, arg_scs)) {
    ERROR("Error UE DL");
    return SRSRAN_ERROR;
  }

  if (srsran_ue_dl_nr_set_carrier_nrscope(&ue_dl, &args_t.base_carrier, arg_scs)) {
    ERROR("Error setting SCH NR carrier");
    return SRSRAN_ERROR;
  }

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
  ue_sync_nr_args.max_srate_hz    = srsran_searcher_args_t.max_srate_hz;
  ue_sync_nr_args.min_scs         = srsran_searcher_args_t.ssb_min_scs;
  ue_sync_nr_args.nof_rx_channels = 1;
  ue_sync_nr_args.disable_cfo     = false;
  ue_sync_nr_args.pbch_dmrs_thr   = 0.5;
  ue_sync_nr_args.cfo_alpha       = 0.1;
  ue_sync_nr_args.recv_obj        = radio.get();
  ue_sync_nr_args.recv_callback   = slot_sync_recv_callback;

  if (srsran_ue_sync_nr_init(&ue_sync_nr, &ue_sync_nr_args) < SRSRAN_SUCCESS) {
    std::cout << "Error initiating UE SYNC NR object" << std::endl;
    logger.error("Error initiating UE SYNC NR object");
    return SRSRAN_ERROR;
  }

  // Be careful of all the frequency setting (SSB/center downlink and etc.)!
  ssb_cfg.srate_hz       = args_t.srate_hz;
  ssb_cfg.center_freq_hz = cs_args.ssb_freq_hz; //args_t.base_carrier.dl_center_frequency_hz; come on!
  ssb_cfg.ssb_freq_hz    = cs_args.ssb_freq_hz;
  ssb_cfg.scs            = cs_args.ssb_scs;
  ssb_cfg.pattern        = cs_args.ssb_pattern;
  ssb_cfg.duplex_mode    = cs_args.duplex_mode;
  ssb_cfg.periodicity_ms = 20; // for all in FR1

  sync_cfg.N_id = cs_ret.ssb_res.N_id;
  sync_cfg.ssb = ssb_cfg;
  sync_cfg.ssb.srate_hz = args_t.srate_hz;
  if (srsran_ue_sync_nr_set_cfg(&ue_sync_nr, &sync_cfg) < SRSRAN_SUCCESS) {
    printf("SYNC: failed to set cell configuration for N_id %d", sync_cfg.N_id);
    logger.error("SYNC: failed to set cell configuration for N_id %d", sync_cfg.N_id);
    return SRSRAN_ERROR;
  }

  return NR_SUCCESS;
}

int Radio::SIB1Loop(){
  std::cout << "SIB1 Loop Starts..." << std::endl;       
  char str[1024] = {};
  // downlink thread
  while(true){
    outcome.timestamp = last_rx_time.get(0);
    if (srsran_ue_sync_nr_zerocopy(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return false;
    }
    // If in sync, update slot index. The synced data is stored in rf_buffer_t.to_cf_t()[0]
    if (outcome.in_sync){
      std::cout << "System frame idx: " << outcome.sfn << std::endl;
      std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;

      // Actual decode
      for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
        srsran_slot_cfg_t slot = {0};
        slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;
        srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);     // Processing for each slot
        if((coreset0_args_t.sfn_c == 0 && outcome.sfn % 2 == 0) || 
           (coreset0_args_t.sfn_c == 1 && outcome.sfn % 2 == 1)) {
          if((outcome.sf_idx) == (uint32_t)(coreset0_args_t.n_0 / 2) || 
             (outcome.sf_idx) == (uint32_t)(coreset0_args_t.n_0 / 2 + 1)){
            // Check the fft plan and how does it manipulate the buffer
            srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);
            std::cout << "ue_dl.pdsch.carrier.dl_center_frequency_hz: " << 
              ue_dl.pdsch.carrier.dl_center_frequency_hz << std::endl;
            // Blind search
            int nof_found_dci = srsran_ue_dl_nr_find_dl_dci(&ue_dl, &slot, 0xFFFF, 
                                                            srsran_rnti_type_si, &dci_1_0_coreset0, 1);
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
            srsran_dci_dl_nr_to_str(&(ue_dl.dci), &dci_1_0_coreset0, str, (uint32_t)sizeof(str));
            printf("Found DCI: %s\n", str);

            std::cout << "base_carrier.pci: " << args_t.base_carrier.pci << std::endl;
            std::cout << "base_carrier.dl_center_frequency_hz: " << args_t.base_carrier.dl_center_frequency_hz << std::endl;
            std::cout << "base_carrier.ul_center_frequency_hz: " << args_t.base_carrier.ul_center_frequency_hz << std::endl;
            std::cout << "base_carrier.ssb_center_freq_hz: " << args_t.base_carrier.ssb_center_freq_hz << std::endl;
            std::cout << "base_carrier.offset_to_carrier: " << args_t.base_carrier.offset_to_carrier << std::endl;
            std::cout << "base_carrier.scs: " << args_t.base_carrier.scs << std::endl;
            std::cout << "base_carrier.nof_prb: " << args_t.base_carrier.nof_prb << std::endl;
            std::cout << "base_carrier.start: " << args_t.base_carrier.start << std::endl;
            std::cout << "base_carrier.max_mimo_layers: " << args_t.base_carrier.max_mimo_layers << std::endl;
            
            srsran_sch_cfg_nr_t pdsch_cfg = {};
            if (srsran_ra_dl_dci_to_grant_nr(&(args_t.base_carrier), &slot, &pdsch_hl_cfg, 
                &dci_1_0_coreset0, &pdsch_cfg, &pdsch_cfg.grant) < SRSRAN_SUCCESS) {
              ERROR("Error decoding PDSCH search");
              return SRSRAN_ERROR;
            }
            srsran_sch_cfg_nr_info(&pdsch_cfg, str, (uint32_t)sizeof(str));
            printf("PDSCH_cfg:\n%s", str);
            pdsch_cfg.grant.tb[0].softbuffer.rx = &softbuffer; // Set softbuffer
            srsran_pdsch_res_nr_t pdsch_res = {}; // Prepare PDSCH result
            pdsch_res.tb[0].payload = data_pdcch;
 
            // Decode PDSCH
            if (srsran_ue_dl_nr_decode_pdsch(&ue_dl, &slot, &pdsch_cfg, &pdsch_res) < SRSRAN_SUCCESS) {
              printf("Error decoding PDSCH search\n");
              continue;
            }
            if (!pdsch_res.tb[0].crc) {
              printf("Error decoding PDSCH (CRC)\n");
              continue;
            }
            printf("Decoded PDSCH (%d B)\n", pdsch_cfg.grant.tb[0].tbs / 8);
            srsran_vec_fprint_byte(stdout, pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8);

            // init_asn_decoder("sample.sib");  // Add ASN decoder
            // push_asn_payload(pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8, SIB_5G, slot.idx);

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
            }
            std::cout << "Decoding SIB 1..." << std::endl;
            asn1::rrc_nr::bcch_dl_sch_msg_s dlsch_msg;
            asn1::cbit_ref dlsch_bref(pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8);
            asn1::SRSASN_CODE err = dlsch_msg.unpack(dlsch_bref);
            sib1 = dlsch_msg.msg.c1().sib_type1();
            std::cout << "SIB 1 Decoded." << std::endl;
            // return NR_SUCCESS;
          // }
          } 
        }
      } 
    } 
  }
  return NR_SUCCESS;
}

// Maybe not necessary
int Radio::MSG1Loop(){
  std::cout << "MSG1 Loop starts..." << std::endl;

  while(true){
    outcome.timestamp = last_rx_time.get(0);
    
    // keep the downlink sync
    if (srsran_ue_sync_nr_zerocopy_nrscope(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome, 
                                           rx_uplink_buffer) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return false;
    }
    
    // rx_uplink_buffer, detect the preambles and get RA-RNTI
    

    // Downlink sync
    if (outcome.in_sync){
      std::cout << "System frame idx: " << outcome.sfn << std::endl;
      std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;
      for(uint32_t i = 0; i < rach_uplink.prach_cfg_nr.nof_subframe_number; i++){
        if(outcome.sf_idx == rach_uplink.prach_cfg_nr.subframe_number[i]){
          std::cout << "Preamble should be in this subframe." << std::endl;
          for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
            srsran_slot_cfg_t slot = {0};
            slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;
            // Processing for each slot
            srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);
            // srsran_prach_detect(&(rach_uplink.prach), outcome.cfo_hz, rx_buffer, slot_sz, );
          }
        }
      }
    }
  }

  return NR_SUCCESS;
}

int Radio::MSG2Loop(){
  // std::cout << "Bypassing SIB1 loop..." << std::endl;
  // uint8_t msg[] = {0x74, 0x81, 0x01, 0x70, 0x10, 0x4c, 0x46, 0x90, 0x80, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x33, 0x61,
  //                  0x80, 0x56, 0x10, 0x50, 0x03, 0x00, 0x00, 0x04, 0x64, 0xc6, 0xb6, 0xc6, 0x1b, 0x37, 0x04, 0x02, 
  //                  0x00, 0x00, 0x08, 0x08, 0x00, 0x04, 0x1a, 0x03, 0x52, 0x42, 0x60, 0x82, 0x81, 0xeb, 0x5c, 0x80, 
  //                  0x00, 0x08, 0xc9, 0xc6, 0xb6, 0xc6, 0x40, 0x01, 0x00, 0x80, 0xcc, 0x95, 0x61, 0xc0, 0x09, 0x39, 
  //                  0xc4, 0x1b, 0x8a, 0x37, 0x18, 0x6e, 0x38, 0xdf, 0x7e, 0xad, 0x8e, 0x1d, 0x40, 0x14, 0xb0, 0x18, 
  //                  0x00, 0x60, 0xc0, 0x60, 0x01, 0x82, 0xc5, 0xb4, 0x61, 0x40, 0x00, 0x00};
  // asn1::rrc_nr::bcch_dl_sch_msg_s dlsch_msg;
  // asn1::cbit_ref dlsch_bref(msg, sizeof(msg));
  // asn1::SRSASN_CODE err = dlsch_msg.unpack(dlsch_bref);
  // sib1 = dlsch_msg.msg.c1().sib_type1();
  
  std::cout << "MSG2 Loop starts..." << std::endl;
  rach_uplink.RachUpInit(sib1, args_t.base_carrier);

  char str[1024] = {};

  // Use a set of new buffer
  ra_rnti.reserve(rach_uplink.prach_cfg_nr.nof_subframe_number);

  // ra_rnti = 1 + s_id + 14 * t_id + 14 * 80 * f_id + 14 * 80 * 8 * ul_carrier
  // s_id = rach_uplink.prach_cfg_nr.starting_symbol;
  // t_id = rach_uplink.prach_cfg_nr.subframe_number[0]; one or many
  // f_id = ?
  // ul_carrier_id = 0 for normal carrier and 1 for SUL carrier
  if (args_t.duplex_mode == SRSRAN_DUPLEX_MODE_SUL){
    for(uint32_t i = 0; i < rach_uplink.prach_cfg_nr.nof_subframe_number; i++){
      ra_rnti[i] = 1 + rach_uplink.prach_cfg_nr.starting_symbol + 
            14 * rach_uplink.prach_cfg_nr.subframe_number[i] + 
            14 * 80 * 0 + 14 * 80 * 8 * 1;
    }  
  }else{
    for(uint32_t i = 0; i < rach_uplink.prach_cfg_nr.nof_subframe_number; i++){
      ra_rnti[i] = 1 + rach_uplink.prach_cfg_nr.starting_symbol + 
            14 * rach_uplink.prach_cfg_nr.subframe_number[i] + 
            14 * 80 * 0 + 14 * 80 * 8 * 0;
    }  
  }
  // ra_rnti[0] = 0x79;
  
  ra_search_space = &pdcch_cfg.search_space[1];
  pdcch_cfg.search_space_present[1]   = true;
  ra_search_space->id                    = 1;
  ra_search_space->coreset_id            = 0;
  ra_search_space->type                  = srsran_search_space_type_common_1;
  ra_search_space->formats[0]            = srsran_dci_format_nr_1_0;
  ra_search_space->nof_formats           = 1;
  
  // set ra search space directly from the SIB 1
  ra_search_space->nof_candidates[0] = sib1.serving_cell_cfg_common.dl_cfg_common.
                                       init_dl_bwp.pdcch_cfg_common.setup().common_search_space_list[0].
                                       nrof_candidates.aggregation_level1;
  ra_search_space->nof_candidates[1] = sib1.serving_cell_cfg_common.dl_cfg_common.
                                       init_dl_bwp.pdcch_cfg_common.setup().common_search_space_list[0].
                                       nrof_candidates.aggregation_level2;
  ra_search_space->nof_candidates[2] = sib1.serving_cell_cfg_common.dl_cfg_common.
                                       init_dl_bwp.pdcch_cfg_common.setup().common_search_space_list[0].
                                       nrof_candidates.aggregation_level4;
  ra_search_space->nof_candidates[3] = sib1.serving_cell_cfg_common.dl_cfg_common.
                                       init_dl_bwp.pdcch_cfg_common.setup().common_search_space_list[0].
                                       nrof_candidates.aggregation_level8;
  ra_search_space->nof_candidates[4] = sib1.serving_cell_cfg_common.dl_cfg_common.
                                       init_dl_bwp.pdcch_cfg_common.setup().common_search_space_list[0].
                                       nrof_candidates.aggregation_level16;

  //  Configure RA search space
  pdcch_cfg.ra_search_space_present = true;
  pdcch_cfg.ra_search_space         = *ra_search_space;
  pdcch_cfg.ra_search_space.type    = srsran_search_space_type_common_1;

  if (srsran_ue_dl_nr_set_pdcch_config(&ue_dl, &pdcch_cfg, &dci_cfg)) {
    ERROR("Error setting CORESET");
    return SRSRAN_ERROR;
  }

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

  while(true){
    outcome.timestamp = last_rx_time.get(0);

    if (srsran_ue_sync_nr_zerocopy(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return false;
    }

    if (outcome.in_sync){
      std::cout << "System frame idx: " << outcome.sfn << std::endl;
      std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;

      for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
        srsran_slot_cfg_t slot = {0};
        slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;

        // Processing for each slot
        srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);

        srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);
        // should be improved to support multiple ra_rntis settings.
        int nof_found_dci = srsran_ue_dl_nr_find_dl_dci(&ue_dl, &slot, ra_rnti[0], srsran_rnti_type_ra, &dci_1_0_coreset0, 1);
        if (nof_found_dci < SRSRAN_SUCCESS) {
          ERROR("Error in blind search");
          return SRSRAN_ERROR;
        }

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

        srsran_dci_dl_nr_to_str(&(ue_dl.dci), &dci_1_0_coreset0, str, (uint32_t)sizeof(str));
        printf("Found DCI: %s\n", str);

        srsran_sch_cfg_nr_t pdsch_cfg = {};
        
        if (srsran_ra_dl_dci_to_grant_nr(&(args_t.base_carrier), &slot, &pdsch_hl_cfg, &dci_1_0_coreset0, &pdsch_cfg, &pdsch_cfg.grant) <
            SRSRAN_SUCCESS) {
          ERROR("Error decoding PDSCH search");
          return SRSRAN_ERROR;
        }

        srsran_sch_cfg_nr_info(&pdsch_cfg, str, (uint32_t)sizeof(str));
        printf("PDSCH_cfg:\n%s", str);

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

        printf("Decoded PDSCH (%d B)\n", pdsch_cfg.grant.tb[0].tbs / 8);
        srsran_vec_fprint_byte(stdout, pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8);

        if (!pdsch_res.tb[0].crc) {
          printf("Error decoding PDSCH (CRC)\n");
          continue;
          // return SRSRAN_ERROR;
        }

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
        }

        std::cout << "Decoding RAR..." << std::endl;
        if (!rar_pdu.unpack(pdsch_res.tb[0].payload, pdsch_cfg.grant.tb[0].tbs / 8)) {
          logger.warning("Error unpacking RAR PDU");
          // return NR_FAILURE;
        }
        tc_rnti = rar_pdu.get_subpdus()[0].get_temp_crnti();
        return NR_SUCCESS;
      }
    }
  }

  return NR_SUCCESS;
}

// Not enabled currently, maybe not necessary
int Radio::MSG3Loop(){
  std::cout << "MSG3 Loop starts..." << std::endl;

  while(true){
    outcome.timestamp = last_rx_time.get(0);
    
    if (srsran_ue_sync_nr_zerocopy_nrscope(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome, 
                                           rx_uplink_buffer) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return false;
    }
    
    // lock_radio_nr.lock();
    // rach_uplink.PushData(tmp_buffer);
    // lock_radio_nr.unlock();

    // if (outcome.in_sync){
    //   std::cout << "System frame idx: " << outcome.sfn << std::endl;
    //   std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;
    // }
  }

  return NR_SUCCESS;
}

int Radio::MSG4Loop(){
  std::cout << "MSG4 Loop starts..." << std::endl;

  char str[1024] = {};

  pdcch_cfg.search_space_present[0]      = true;
  pdcch_cfg.search_space[0].id           = 2;
  pdcch_cfg.search_space[0].coreset_id   = 0;
  pdcch_cfg.search_space[0].type         = srsran_search_space_type_common_1;
  pdcch_cfg.search_space[0].formats[0]   = srsran_dci_format_nr_1_0;
  pdcch_cfg.search_space[0].nof_formats  = 1;

  // coreset0_t.duration = 1;
  pdcch_cfg.coreset[0] = coreset0_t; 
  
  // set search space with TC-RNTI
  for (uint32_t L = 0; L < SRSRAN_SEARCH_SPACE_NOF_AGGREGATION_LEVELS_NR; L++) {
    pdcch_cfg.search_space[0].nof_candidates[L] = ra_search_space->nof_candidates[L];
  }

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

  while(true){
    outcome.timestamp = last_rx_time.get(0);
    if (srsran_ue_sync_nr_zerocopy(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return NR_FAILURE;
    }
    
    // If in sync, update slot index. The synced data is stored in rf_buffer_t.to_cf_t()[0]
    if (outcome.in_sync){
      std::cout << "System frame idx: " << outcome.sfn << std::endl;
      std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;
  
      for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
        srsran_slot_cfg_t slot = {0};
        slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;

        // Processing for each slot
        srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);

        srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);
        // Blind search
        int nof_found_dci = srsran_ue_dl_nr_find_dl_dci(&ue_dl, &slot, tc_rnti, srsran_rnti_type_tc, &dci_1_0_coreset0, 1);
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

        srsran_dci_dl_nr_to_str(&(ue_dl.dci), &dci_1_0_coreset0, str, (uint32_t)sizeof(str));
        printf("Found DCI: %s\n", str);
        srsran_sch_cfg_nr_t pdsch_cfg = {};
        if (srsran_ra_dl_dci_to_grant_nr(&(args_t.base_carrier), &slot, &pdsch_hl_cfg, &dci_1_0_coreset0, 
                                         &pdsch_cfg, &pdsch_cfg.grant) < SRSRAN_SUCCESS) {
          ERROR("Error decoding PDSCH search");
          return SRSRAN_ERROR;
        }

        srsran_sch_cfg_nr_info(&pdsch_cfg, str, (uint32_t)sizeof(str));
        printf("PDSCH_cfg:\n%s", str);
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

        printf("Decoded PDSCH (%d B)\n", pdsch_cfg.grant.tb[0].tbs / 8 - 10);
        srsran_vec_fprint_byte(stdout, pdsch_res.tb[0].payload + 10, pdsch_cfg.grant.tb[0].tbs / 8);

        if (!pdsch_res.tb[0].crc) {
          printf("Error decoding PDSCH (CRC)\n");
          continue;
          // return SRSRAN_ERROR;
        }

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
        }

        std::cout << "Decoding Msg 4..." << std::endl;
        asn1::rrc_nr::dl_ccch_msg_s dlcch_msg;
        // What the first 10 bytes are? 
        asn1::cbit_ref dlcch_bref(pdsch_res.tb[0].payload + 10, pdsch_cfg.grant.tb[0].tbs / 8 - 10);
        asn1::SRSASN_CODE err = dlcch_msg.unpack(dlcch_bref);
        if (err != asn1::SRSASN_SUCCESS) {
          ERROR("Failed to unpack DL-CCCH message (%d B)", pdsch_cfg.grant.tb[0].tbs / 8 - 10);
        }

        rrc_setup = dlcch_msg.msg.c1().rrc_setup();
        // rrc_setup_ies_s& setup_ies = rrc_setup.crit_exts.set_rrc_setup();
        std::cout << "Msg 4 Decoded." << std::endl;
        switch (dlcch_msg.msg.c1().type().value) {
          case asn1::rrc_nr::dl_ccch_msg_type_c::c1_c_::types::rrc_reject: {
            std::cout << "Unfortunately, it's a rrc_reject ;(" << std::endl;
          }break;
          case asn1::rrc_nr::dl_ccch_msg_type_c::c1_c_::types::rrc_setup: {
            std::cout << "It's a rrc_setup, hooray!" << std::endl;
            printf("rrc-TransactionIdentifier: %u\n", rrc_setup.rrc_transaction_id);
            // printf("criticalExtensions: \n", rrc_setup.crit_exts);
          }break;
          default:
            std::cout << "none detected" << std::endl;
          break;
        }
        asn1::json_writer js_msg4;
        rrc_setup.to_json(js_msg4);
        printf("rrcSetup content: %s\n", js_msg4.to_string().c_str());
        asn1::cbit_ref bref_cg(rrc_setup.crit_exts.rrc_setup().master_cell_group.data(),
                         rrc_setup.crit_exts.rrc_setup().master_cell_group.size());
        if (master_cell_group.unpack(bref_cg) != asn1::SRSASN_SUCCESS) {
          ERROR("Could not unpack master cell group config.");
        }        
        asn1::json_writer js;
        master_cell_group.to_json(js);
        printf("masterCellGroup: %s\n", js.to_string().c_str());
        if (!master_cell_group.sp_cell_cfg.recfg_with_sync.new_ue_id){
          c_rnti = tc_rnti;
        }
        std::cout << "c-rnti: " << master_cell_group.sp_cell_cfg.recfg_with_sync.new_ue_id << std::endl;
        return NR_SUCCESS;
      } 
    }
  }
  return NR_SUCCESS;
}

int Radio::DCILoop(){
  std::cout << "DCI Loop starts..." << std::endl;

  char str[1024] = {};

  pdcch_cfg.search_space_present[0]      = true;
  pdcch_cfg.search_space_present[1]      = false;
  pdcch_cfg.search_space_present[2]      = false;
  pdcch_cfg.ra_search_space_present      = false;
  pdcch_cfg.search_space[0].id           = 2;
  pdcch_cfg.search_space[0].coreset_id   = 1;
  pdcch_cfg.search_space[0].type         = srsran_search_space_type_ue;
  pdcch_cfg.search_space[0].formats[0]   = srsran_dci_format_nr_1_1; // from RRCSetup
  pdcch_cfg.search_space[0].formats[1]   = srsran_dci_format_nr_0_1; // from RRCSetup
  pdcch_cfg.search_space[0].nof_formats  = 2;
  pdcch_cfg.coreset[0] = coreset0_t; 

  // all the Coreset information is from RRCSetup
  srsran_coreset_t coreset1_t = {}; 
  coreset1_t.id = 1; 
  coreset1_t.duration = 2;
  for(int i = 0; i < 45; i++){
    coreset1_t.freq_resources[i] = coreset0_t.freq_resources[i]; //master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.
                              //init_dl_bwp.pdcch_cfg.setup().ctrl_res_set_to_add_mod_list[0].freq_domain_res;
  }
  coreset1_t.offset_rb = 0;
  coreset1_t.precoder_granularity = srsran_coreset_precoder_granularity_reg_bundle;
  coreset1_t.reg_bundle_size = srsran_coreset_bundle_size_n6;
  coreset1_t.mapping_type = srsran_coreset_mapping_type_non_interleaved; 
  coreset1_t.dmrs_scrambling_id_present = false;
  coreset1_t.interleaver_size = srsran_coreset_bundle_size_n2;
  coreset1_t.shift_index = 0;
  pdcch_cfg.coreset[1] = coreset1_t;
  pdcch_cfg.coreset_present[1] = true;

  // char coreset_info[512] = {};
  // srsran_coreset_to_str(&coreset1_t, coreset_info, sizeof(coreset_info));
  // printf("Coreset parameter: %s", coreset_info);

  // For FR1 offset_to_point_a uses prbs with 15kHz scs. 
  double pointA = srsran_searcher_cfg_t.ssb_freq_hz - (SRSRAN_SSB_BW_SUBC / 2) *
    cell.abs_ssb_scs - cell.k_ssb * SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz) 
    - sib1.serving_cell_cfg_common.dl_cfg_common.freq_info_dl.offset_to_point_a * 
    SRSRAN_SUBC_SPACING_NR(srsran_subcarrier_spacing_15kHz) * NRSCOPE_NSC_PER_RB_NR;

  double coreset1_center_freq_hz = pointA + srsran_coreset_get_bw(&coreset1_t) / 2 * 
    cell.abs_pdcch_scs * NRSCOPE_NSC_PER_RB_NR;
  std::cout << "previous offset: " << arg_scs.coreset_offset_scs << std::endl;
  arg_scs.coreset_offset_scs = (cs_args.ssb_freq_hz - coreset1_center_freq_hz) / cell.abs_pdcch_scs;
  std::cout << "current offset: " << arg_scs.coreset_offset_scs << std::endl;
  // arg_scs.coreset_offset_scs = -25;
  
   // set ra search space directly from the SIB 1
  pdcch_cfg.search_space[0].nof_candidates[0] = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.
                                                pdcch_cfg.setup().search_spaces_to_add_mod_list[0].
                                                nrof_candidates.aggregation_level1;
  pdcch_cfg.search_space[0].nof_candidates[1] = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.
                                                pdcch_cfg.setup().search_spaces_to_add_mod_list[0].
                                                nrof_candidates.aggregation_level2;
  pdcch_cfg.search_space[0].nof_candidates[2] = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.
                                                pdcch_cfg.setup().search_spaces_to_add_mod_list[0].
                                                nrof_candidates.aggregation_level4;
  pdcch_cfg.search_space[0].nof_candidates[3] = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.
                                                pdcch_cfg.setup().search_spaces_to_add_mod_list[0].
                                                nrof_candidates.aggregation_level8;
  pdcch_cfg.search_space[0].nof_candidates[4] = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.
                                                pdcch_cfg.setup().search_spaces_to_add_mod_list[0].
                                                nrof_candidates.aggregation_level16;

  /// Format 0_1 specific configuration (for PUSCH only)
  dci_cfg.nof_ul_bwp = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.ul_bwp_to_add_mod_list.size(); ///< Number of UL BWPs excluding the initial UL BWP, mentioned in the TS as N_BWP_RRC
  dci_cfg.nof_ul_time_res = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.init_ul_bwp.pusch_cfg.setup().
                            pusch_time_domain_alloc_list_present ? 
                            master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.init_ul_bwp.pusch_cfg.setup().
                            pusch_time_domain_alloc_list.setup().size() : 0;     ///< Number of dedicated PUSCH time domain resource assigment, set to 0 for default
  dci_cfg.nof_srs = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.init_ul_bwp.srs_cfg_present ? 
                    master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.init_ul_bwp.srs_cfg.setup().
                    srs_res_to_add_mod_list.size() : 0;             ///< Number of configured SRS resources
  dci_cfg.nof_ul_layers = 1;       ///< Set to the maximum number of layers for PUSCH
  dci_cfg.pusch_nof_cbg = 0;
         ///< determined by maxCodeBlockGroupsPerTransportBlock for PUSCH
  dci_cfg.report_trigger_size = 0; ///< determined by reportTriggerSize
  dci_cfg.enable_transform_precoding = false;      ///< Set to true if PUSCH transform precoding is enabled
  dci_cfg.dynamic_dual_harq_ack_codebook = true;  ///< Set to true if HARQ-ACK codebook is set to dynamic with 2 sub-codebooks
  dci_cfg.pusch_tx_config_non_codebook = false;    ///< Set to true if PUSCH txConfig is set to non-codebook
  dci_cfg.pusch_ptrs = false;                      ///< Set to true if PT-RS are enabled for PUSCH transmission
  dci_cfg.pusch_dynamic_betas = false;             ///< Set to true if beta offsets operation is not semi-static
  dci_cfg.pusch_alloc_type = srsran_resource_alloc_type1; ///< PUSCH resource allocation type
  dci_cfg.pusch_dmrs_type = srsran_dmrs_sch_type_1;  ///< PUSCH DMRS type
  dci_cfg.pusch_dmrs_max_len = srsran_dmrs_sch_len_1; ///< PUSCH DMRS maximum length

  /// Format 1_1 specific configuration (for PDSCH only)
  dci_cfg.harq_ack_codebok       = srsran_pdsch_harq_ack_codebook_dynamic;
  dci_cfg.nof_dl_bwp             = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.dl_bwp_to_add_mod_list.size();
  dci_cfg.nof_dl_time_res        = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   pdsch_time_domain_alloc_list_present ? 
                                   master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   pdsch_time_domain_alloc_list.setup().size() : 0; // yes
  dci_cfg.nof_aperiodic_zp       = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   aperiodic_zp_csi_rs_res_sets_to_add_mod_list.size(); // yes
  dci_cfg.pdsch_nof_cbg          = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   max_nrof_code_words_sched_by_dci_present ? 
                                   master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   max_nrof_code_words_sched_by_dci : 0;
  dci_cfg.nof_dl_to_ul_ack       = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.ul_cfg.init_ul_bwp.pucch_cfg.setup().
                                   dl_data_to_ul_ack.size();
  dci_cfg.pdsch_inter_prb_to_prb = master_cell_group.sp_cell_cfg.sp_cell_cfg_ded.init_dl_bwp.pdsch_cfg.setup().
                                   vrb_to_prb_interleaver_present;
  dci_cfg.pdsch_rm_pattern1      = false;
  dci_cfg.pdsch_rm_pattern2      = false;
  dci_cfg.pdsch_2cw              = false;
  dci_cfg.multiple_scell         = false; // yes
  dci_cfg.pdsch_tci              = false; // yes
  dci_cfg.pdsch_cbg_flush        = false; // yes
  dci_cfg.pdsch_dynamic_bundling = false; // yes
  dci_cfg.pdsch_alloc_type       = srsran_resource_alloc_type1; // yes
  dci_cfg.pdsch_dmrs_type        = srsran_dmrs_sch_type_1; // yes
  dci_cfg.pdsch_dmrs_max_len     = srsran_dmrs_sch_len_1; // yes

  pdsch_hl_cfg.alloc = dci_cfg.pdsch_alloc_type;

  dci_cfg.bwp_dl_initial_bw   = sib1.serving_cell_cfg_common.dl_cfg_common.freq_info_dl.scs_specific_carrier_list[0].carrier_bw;//srsran_coreset_get_bw(&coreset0_t);
  dci_cfg.bwp_dl_active_bw    = sib1.serving_cell_cfg_common.dl_cfg_common.freq_info_dl.scs_specific_carrier_list[0].carrier_bw;//srsran_coreset_get_bw(&coreset0_t);
  if(args_t.duplex_mode == SRSRAN_DUPLEX_MODE_TDD){
    dci_cfg.bwp_ul_initial_bw   = dci_cfg.bwp_dl_initial_bw;
    dci_cfg.bwp_ul_active_bw    = dci_cfg.bwp_dl_active_bw;
  } else{
    ERROR("FDD not implemented for DCI bwp allocation");
    dci_cfg.bwp_ul_initial_bw = dci_cfg.bwp_dl_initial_bw;
    dci_cfg.bwp_ul_active_bw    = dci_cfg.bwp_dl_active_bw;
  }

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

  while(true){
    outcome.timestamp = last_rx_time.get(0);
    if (srsran_ue_sync_nr_zerocopy(&ue_sync_nr, rf_buffer_t.to_cf_t(), &outcome) < SRSRAN_SUCCESS) {
      std::cout << "SYNC: error in zerocopy" << std::endl;
      logger.error("SYNC: error in zerocopy");
      return NR_FAILURE;
    }
    
    // If in sync, update slot index. The synced data is stored in rf_buffer_t.to_cf_t()[0]
    if (outcome.in_sync){
      std::cout << "System frame idx: " << outcome.sfn << std::endl;
      std::cout << "Subframe idx: " << outcome.sf_idx << std::endl;
  
      for(int slot_idx = 0; slot_idx < SRSRAN_NOF_SLOTS_PER_SF_NR(arg_scs.scs); slot_idx++){
        srsran_slot_cfg_t slot = {0};
        slot.idx = (outcome.sf_idx) * SRSRAN_NSLOTS_PER_FRAME_NR(arg_scs.scs) / 10 + slot_idx;

        // Processing for each slot
        srsran_vec_cf_copy(rx_buffer, rx_buffer + slot_idx*slot_sz, slot_sz);
        srsran_ue_dl_nr_estimate_fft_nrscope(&ue_dl, &slot, arg_scs);

        int nof_found_dci = srsran_ue_dl_nr_find_dl_dci(&ue_dl, &slot, c_rnti, srsran_rnti_type_c, &dci_1_0_coreset0, 1);
        if (nof_found_dci < SRSRAN_SUCCESS) {
          ERROR("Error in blind search");
          return SRSRAN_ERROR;
        }

        srsran_dci_ul_nr_t dci_0_1;
        int nof_ul_dci = srsran_ue_dl_nr_find_ul_dci(&ue_dl, &slot, c_rnti, srsran_rnti_type_c, &dci_0_1, 1);

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

        if (nof_found_dci < 1 && nof_ul_dci < 1) {
          printf("No DCI found :'(\n");
          continue;
        }

        if(nof_found_dci > 0){
          srsran_dci_dl_nr_to_str(&(ue_dl.dci), &dci_1_0_coreset0, str, (uint32_t)sizeof(str));
          printf("Found DCI: %s\n", str);
          if(dci_1_0_coreset0.ctx.format == srsran_dci_format_nr_1_1 || 
            dci_1_0_coreset0.ctx.format == srsran_dci_format_nr_1_0){
            srsran_sch_cfg_nr_t pdsch_cfg = {};
            if (srsran_ra_dl_dci_to_grant_nr(&(args_t.base_carrier), &slot, &pdsch_hl_cfg, &dci_1_0_coreset0, 
                                            &pdsch_cfg, &pdsch_cfg.grant) < SRSRAN_SUCCESS) {
              ERROR("Error decoding PDSCH search");
              return SRSRAN_ERROR;
            }

            srsran_sch_cfg_nr_info(&pdsch_cfg, str, (uint32_t)sizeof(str));
            printf("PDSCH_cfg:\n%s", str);
          }
        }
        
        if(nof_ul_dci > 0){
          srsran_dci_ul_nr_to_str(&(ue_dl.dci), &dci_0_1, str, (uint32_t)sizeof(str));
          printf("Found DCI: %s\n", str);
        }
      } 
    }
  }
  return NR_SUCCESS;
}

