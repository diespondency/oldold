#include <mutex>
#include <iostream>

#include "srsran/config.h"
#include "srsran/asn1/rrc_nr.h"
#include "srsran/asn1/asn1_utils.h"
#include "srsran/phy/phch/prach.h"
#include "srsran/common/band_helper.h"

#include "nrscope/hdr/nrscope_def.h"

// typedef struct {
//   bool is_sib1_found = false;
//   bool is_msg1_found = false;
//   bool is_msg2_found = false;
//   bool is_msg3_found = false;
//   bool is_msg4_found = false;

//   //msg content definition.
// }nrscope_dl_ul_exchange;


class RachUplink{
  public:
    // bool new_subframe_flag;
    asn1::rrc_nr::sib1_s sib1; 
    srsran_carrier_nr_t base_carrier;
    // nrscope_dl_ul_exchange rach_dl_ul_info;
    prach_nr_config_t prach_cfg_nr;
    srsran_prach_t prach;
    srsran_prach_cfg_t prach_cfg;
    


    RachUplink();
    ~RachUplink();
    int RachUpInit(asn1::rrc_nr::sib1_s sib1_input, srsran_carrier_nr_t carrier_input);

    int Msg1Decode(cf_t* signal);
    int Msg3Decode();
};