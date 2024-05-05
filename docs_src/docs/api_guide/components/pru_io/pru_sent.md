# SENT {#SENT}

[TOC]

## Introduction

The SENT receiver firmware running on PRU-ICSS provides an interface to execute the SENT protocol. The SENT application interacts with the SENT receiver firmware interface. The SENT firmware and examples are based on GPIO interface from PRU-ICSS.

# Feature Supported
<table>
<tr><th>Feature                             </th><th> Support </th><th> Remarks</th></tr>
<tr><td rowspan="2"> No. of Channels        </td><td> 8       </td><td> Can support upto 8 channels using default example using PRU GPIO based capture </td></tr>
<tr>                                             <td> 6       </td><td> Can support upto 6 channels in PRUICSS IEP CAP example (Note : It can be extendend to 8 channels using PRUICSS ECAP) </td></tr>
<tr><td> Min Tick Period                    </td><td> 500ns   </td><td> Experimental support       </td></tr>
<tr><td> Sync Correction                    </td><td> Yes     </td><td> 20% tolerance value </td></tr>
<tr><td> Frame buffering                    </td><td> No      </td><td> Supports only Single frame buffering </td></tr>
<tr><td> Configurable FIFO                  </td><td> No      </td><td> Will be implemented in future release </td></tr>
<tr><td> Receiving 1 – 6 Data Nibbles       </td><td> Yes     </td><td> Receiving and decoding Supported in Firmware </td></tr>
<tr><td> CRC calculation                    </td><td> Yes     </td><td> CRC calculation done along with data nibble reception </td></tr>
<tr><td> Pause Pulse                        </td><td> No      </td><td> Will be implemented in future release </td></tr>
<tr><td rowspan="2"> Short Serial Message Format </td><td> No      </td><td> Will be implemented in default example using PRU GPIO based capture in future release</td></tr>
<tr>                                             <td> Yes     </td><td> Supported in PRUICSS IEP CAP example  </td></tr>
<tr><td rowspan="2"> Enhanced Serial Message Format </td><td> No      </td><td> Will be implemented in default example using PRU GPIO based capture in future release</td></tr>
<tr>                                             <td> Yes     </td><td> Supported in PRUICSS IEP CAP example  </td></tr>
<tr><td> Successive Calibration Pulse Check </td><td> No      </td><td> Will be implemented in future release </td></tr>
<tr><td> Selectable data length for receive </td><td> No      </td><td> Will be implemented in future release </td></tr>
</table>
## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Syscfg based customization will be supported in future releases.

## SENT Design

\subpage SENT_DESIGN explains the firmware design in detail.

## Example

- \ref EXAMPLES_SENT_DECODER
- \ref EXAMPLES_SENT_DECODER_PRUICSS_IEP_ECAP
