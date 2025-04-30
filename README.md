NR-Scope
========

Implement on top of srsRAN_4G, decode the DCI and SIB information for 5G SA base station.

Files and functions:

```
Entry: /nrscope/src/main.cc
Load config: /nrscope/src/libs/load_config.cc
Radio thread (cell search, mib decoding, coreset decoding, etc.): /nrscope/src/libs/radio_nr.cc
Config file: /nrscope/config/config.yaml
```

Usage:

```
mkdir build
cd build
cmake ../
make -j {nof_proc}
cd nrscope/src/
sudo ./nrscope
```

Logs:

(Aug-9) Solved some problems in synchronization, see how to use external clock to perform the synchronization.

(Aug-11) DCI 1_0 and PDSCH are decoded for gNB in n41 with `ssb_freq=2523.75 MHz`. SIB 1 payload is verified with  `/srsue/src/stack/rrc_nr/test/ue_rrc_nr_test.cc` and [libasn1](https://github.com/j0lama/libasn).

(Aug-13) DCI decoding is tested with the following settings using srsgNB: `n41, TDD, 20MHz, 30kHz SCS`; `n78, TDD, 20MHz, 30kHz SCS`; `n41, TDD, 20MHz, 15kHz SCS`; `n3, FDD, 20MHz, 15kHz SCS`; `n3, FDD, 10MHz, 15kHz SCS` and `n41, TDD, 10MHz, 15kHz SCS`.

(Aug-21) Used all RA-RNTI to decode DCI 1_0 scrambled with RA-RNTI and decoded the RAR (Msg 2) bytes. Then we should use the TC-RNTI in Msg 2 to decode DCI 1_0 for Msg 4 (RRC ConnectionSetup).

(Aug-26) Verified Msg 2 decoding and get TC-RNTI. Msg 4 successfully decoded and for UE tracked in RACH process, we can easily decode its other DCI in the following data communication.

(Sep-2) Decoding DCIs for 1 UE is finished. But some settings that affects DCI's size setting are still not clear to me. They are set manually and should be set according to RRC messages in the future.

(Sep-4) Downlink DCI 1_1 and grants are decoded, and the inconsistency between srsRAN_4G and srsRAN_Project codes causes the DCI size problem. Now the downlink grants are correctly decoded. Uplink DCI 0_1 decoded, but there are some problems left -- the elements of the DCI 0_1 is not set correctly so the uplink grants may have some problems. This problem can be easily solved by more reading, but left for the testing phase.  

(Oct-1) Works for 30kHz SCS with 20MHz bandwidth in TDD bands (n41, n48, n78), both srsgNB and sercomm small cell, some bugs in different SCS and bandwidth settings, such as 15kHz+20MHz, 15kHz+10MHz and 30kHz+10MHz, in TDD band (n41) and FDD bands (n3).

TODOs:

Test the whole system in different band settings.

Thinking about parallel decoding RAR, RRCSetup and DCI in normal data communication.

Design algorithms to decode the UE whose C-RNTI is unknown to us. (Maybe)
