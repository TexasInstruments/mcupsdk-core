The ICSS DUAL EMAC firmware consists of the following files:

micro_scheduler.asm: contains the initialization routine and the main control loop. Performs following tasks:
	PRU register initialization
	Controls that state machine sequence in the main control loop. The following are the 3 states.
            -> Recieve state (Rcv).
                -> perform the Ethernet packets recieve functions.
            -> Transmit state (Xmt).
                -> perform the Ethernet packets transmit functions.
            -> Statistics state (STAT).
                -> perform the statistics calculation for packets drop or recieved.

emac_MII_Rcv.asm: contains the packet recieve routines
	Depending on the current recieve state, call one of the 3 recieve functions.
            -> FN_RCV_FB.
                -> Recieves the first 32 bytes at the SOF of the incoming packet.
            -> FN_RCV_NB.
                -> Recieves the next 32 bytes blocks inbetween SOF and EOF of the incoming packet.
            -> FN_RCV_FB.
                -> Recieves the last 32 bytes at the EOF of the incoming packet.

emac_MII_Xmt.asm: contains the packet transmit routines
	Depending on the current transmit state, call one of the 3 recieve functions.
            -> FN_XMT_scheduler.
                -> Check all the transmit queue for packets if any available starts transmitting the packets.
            -> FN_XMT_queue.
                -> Transmits the first 32 bytes TX L1 fifo .
            -> XMT_queue.
                -> Transmits the next 32 bytes TX L1 fifo.
            -> FN_XMT_LB.
                -> Transmits the last 32 bytes TX L1 fifo.

emac_statistics.asm: increments the stats count for incoming and outgoing packets
	Checks if an stats event is pending.
            -> TX_INCREMENT_STAT_COUNTER
                -> increments the stats count for TX events.
            -> RX_INCREMENT_STAT_COUNTER
                -> increments the stats count for RX events.
	
emac_tts.asm: Perform the TTS (TIME TRIGGERED SENT) function routine
	Depending on the current TTS states does this following functions.
            -> FN_TTS_IEP_CFG_PRE_ICSS_REV1/2.
                -> configures the IEP timer registers.
            -> FN_TTS_IEP_CFG_CLEAR.
                -> clear IEP timer configuration.
            -> FN_TTS_PKT_SIZE_CHECK_ICSS_REV1/2.
                -> Check if the packet to be sent is within the permissible range.
            ->FN_TTS_IEP_CMPCFG_ARBITRATION.
                -> Semphore mechanics for acquiring sharable/common resources.
            ->FN_TTS_EXIT_IEP_CMPCFG_ARBITRATION.
                -> Semphore mechanics for releasing sharable/common resources.


Prior to reviewing the ICSS DUAL EMAC firmware sources, its essenstial to have a good understanding of the PRU-ICSS subsystem and the PRU Assembly instruction set.

In order to gain an understanding of the PRU-ICSSS subsystem please refer to the Technical reference manual for the respective SOC. 

Link to PRU Assembly document: http://processors.wiki.ti.com/index.php/PRU_Assembly_Instructions

Additional information abouthe PRU-ICSS subsystem can be found at: http://processors.wiki.ti.com/index.php/PRU-ICSS



