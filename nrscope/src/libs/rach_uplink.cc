#include "nrscope/hdr/rach_uplink.h"

std::mutex lock_rach;

// struct SubframeData{
//   uint32_t nof_samples;
//   cf_t* uplink_buffer;
//   bool new_data;
// };

// SubframeData subframe_data;

RachUplink::RachUplink(){
  // rach_dl_ul_info = {false, false, false, false, false};
  sib1 = {};
  prach_cfg_nr = {};
  prach = {};
  prach_cfg = {};
}

RachUplink::~RachUplink(){

}

int RachUplink::RachUpInit(asn1::rrc_nr::sib1_s sib1_input, srsran_carrier_nr_t carrier_input){
  // subframe_data.uplink_buffer = srsran_vec_cf_malloc(nof_samples);
  // subframe_data.nof_samples = nof_samples;
  // srsran_vec_zero(subframe_data.uplink_buffer, nof_samples);
  sib1 = sib1_input;
  base_carrier = carrier_input;
  srsran::srsran_band_helper bands;
  uint16_t band = bands.get_band_from_dl_freq_Hz(base_carrier.dl_center_frequency_hz);

  uint32_t cfg_idx = sib1.serving_cell_cfg_common.ul_cfg_common.
    init_ul_bwp.rach_cfg_common.setup().rach_cfg_generic.prach_cfg_idx;
  if(bands.get_duplex_mode(band) == SRSRAN_DUPLEX_MODE_TDD){
    prach_cfg_nr = *srsran_prach_nr_get_cfg_fr1_unpaired(cfg_idx);
  }else if (bands.get_duplex_mode(band) == SRSRAN_DUPLEX_MODE_FDD){
    prach_cfg_nr = *srsran_prach_nr_get_cfg_fr1_paired(cfg_idx);
  }

  // Set the config for prach_cfg
  prach_cfg.is_nr = true;
  prach_cfg.config_idx = cfg_idx;
  if(sib1.serving_cell_cfg_common.ul_cfg_common.init_ul_bwp.rach_cfg_common.setup().prach_root_seq_idx.l839()){
    prach_cfg.root_seq_idx = sib1.serving_cell_cfg_common.ul_cfg_common.init_ul_bwp.
      rach_cfg_common.setup().prach_root_seq_idx.l839();
  }else if(sib1.serving_cell_cfg_common.ul_cfg_common.init_ul_bwp.rach_cfg_common.setup().prach_root_seq_idx.l139()){
      prach_cfg.root_seq_idx = sib1.serving_cell_cfg_common.ul_cfg_common.init_ul_bwp.
        rach_cfg_common.setup().prach_root_seq_idx.l139();
  }
  prach_cfg.zero_corr_zone = sib1.serving_cell_cfg_common.ul_cfg_common.
    init_ul_bwp.rach_cfg_common.setup().rach_cfg_generic.zero_correlation_zone_cfg;
  prach_cfg.freq_offset = sib1.serving_cell_cfg_common.ul_cfg_common.init_ul_bwp.
    rach_cfg_common.setup().rach_cfg_generic.msg1_freq_start;
  prach_cfg.num_ra_preambles = 64;
  prach_cfg.hs_flag = false;
  if(bands.get_duplex_mode(band) == SRSRAN_DUPLEX_MODE_TDD){
    prach_cfg.tdd_config.configured = true;
  }

  srsran_prach_init(&prach, 2048);

  srsran_prach_set_cfg(&prach, &prach_cfg, 
    sib1.serving_cell_cfg_common.ul_cfg_common.freq_info_ul.scs_specific_carrier_list[0].carrier_bw);

  return NR_SUCCESS;
}

int RachUplink::Msg1Decode(cf_t* signal){

  // while(true){
  //   if(subframe_data.new_data){
  //     std::cout << "new data in" << std::endl;

  //     if(rach_dl_ul_info.is_sib1_found){
  //       std::cout << "MSG1 Loop Starts..." << std::endl;
  //       // listen to preamble in PUSCH 

  //     }else{
  //       // if SIB 1 is not decoded, skip
  //       continue;
  //     }
  //     // subframe_data.new_data = false;
  //   }
  // }

  return NR_SUCCESS;
}

int RachUplink::Msg3Decode(){
  // while(true){
  //   if(subframe_data.new_data){
  //     lock_rach.lock();
  //     subframe_data.new_data = false;
  //     lock_rach.unlock();

  //     std::cout << "new data in" << std::endl;
  //     if(rach_dl_ul_info.is_sib1_found){
  //       // listen to preamble in PUSCH 

  //     }else{
  //       // if SIB 1 is not decoded, skip
  //       continue;
  //     }
  //   }
  // }

  return NR_SUCCESS;
}