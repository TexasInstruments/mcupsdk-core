# Ether-Ring Overview {#ETHERRING_OVERVIEW}
[TOC]
Ether-Ring functionality is achieved using hardware configuration with assistance of software. The 3 main components are
- **Packet Duplication** - Inter-vlan Routing(Hardware Configuration)
- **Duplicate Packet Rejection** - Software assistance
- **Ring Termination** - Hardware Configuration

# Software Architecture
\image html etherring_sof_arch.png width=50%

# CAN to Ethernet Traffic Simulation
<div style="display: flex; align-items: center;">
    <div style="flex: 1; text-align: left; max-width: 60%;">
        <h2>Software Assistance:</h2>
        <ul>
            <li><strong>Interrupt and Polling Configuration</strong>: Disabled Packet Transmission Complete Interrupt and Packet Reception Interrupt to reduce interrupt overhead. Implemented periodic polling for packet reception to ensure timely processing.</li>
            <li><strong>Timer Synchronization</strong>: Synchronized timer clock source with 802.1AS to reduce drift in timer periodic interrupts.</li>
            <li><strong>Pulse Generation Configuration</strong>: Configured pulse generation to start before the 802.1Qbv gate opens to ensure accurate and timely pulse generation.</li>
        </ul>
    </div>
    <div style="flex: 1; text-align: right; max-width: 40%;">
        <img src="etherring_can_to_eth.png" alt="CAN to Ethernet" style="width: 80%;">
    </div>
</div>

# Packet Duplication on Transmission
<div style="display: flex; align-items: center;">
    <div style="flex: 1; text-align: left; max-width: 70%;">
        <ul>
            <li><strong>Inter-VLAN routing</strong>: Routes VLAN Tag1 packet to VLAN Tag2 to duplicate packets in both directions.</li>
            <li><strong>VLAN routing configuration</strong>: Configured on both MAC ports to egress packets with updated VLAN tags.</li>
            <li><strong>802.1Q Packet Duplication</strong>: Uses Inter-VLAN routing feature to duplicate packets and send them in both clockwise and anti-clockwise directions in the Ring Network.</li>
            <li><strong>Address Lookup Engine(ALE)</strong>: Determines Inter-VLAN egress opcode for each packet to be VLAN routed.</li>
            <li><strong>Classifier/Policer configuration</strong>: Configured for the route to modify VLAN tags and duplicate Ethernet packet on both MAC ports.</li>
        </ul>
    </div>
    <div style="flex: 1; text-align: right; max-width: 30%;">
        <img src="etherring_pkt_duplication.png" alt="CAN to Ethernet" style="width: 80%;">
    </div>
</div>

# Software Assistance on Transmission side
<div style="display: flex; align-items: center;">
    <div style="flex: 1; text-align: left; max-width: 65%;">
        <ul>
            <li>Adds custom header using Zero-copy which helps to reject the duplicate packets on receiver zonal controller</li>
            <li>Generating Ethernet packet with non continuous buffers for Header addition.</li>
        </ul>
    </div>
    <div style="flex: 1; text-align: right; max-width: 75%;">
        <img src="etherring_tx_assistance.png" alt="CAN to Ethernet" style="width: 90%;">
    </div>
</div>

# Software Assistance on Reception side
<div style="display: flex; align-items: center;">
    <div style="flex: 1; text-align: left; max-width: 65%;">
        <ul>
            <li>Removes Custom Header on the receiver end after detecting and deleting duplicate packets coming from other direction, ensuring clean and original packet delivery</li>
            <li>Generating Ethernet packet with non continuous buffers for Header addition.</li>
        </ul>
    </div>
    <div style="flex: 1; text-align: right; max-width: 75%;">
        <img src="etherring_rx_assistance.png" alt="CAN to Ethernet" style="width: 90%;">
    </div>
</div>

# Duplicate packet Rejection on Reception side(Duplicate Packet Rejection)
<div style="display: flex; align-items: center;">
    <div style="flex: 1; text-align: left; max-width: 65%;">
        <ul>
            <li><strong>Packet Detection Flow</strong>: SequenceID-based duplicate detection using per-node state tracking. Each node maintains last accepted sequence number and 64-bit history bitmap.</li>
            <li><strong>Per-node State</strong>: Tracks recSeqNum (last accepted sequence), seqHistory (64-bit bitmap), and initialization flag for each data stream identified by MAC address.</li>
            <li><strong>Five Detection Cases</strong>:
                <ol>
                    <li>In-Order/Future Packet (seqDiff > 0) - Accept and shift window forward</li>
                    <li>Out-of-Order Within Window (seqDiff < 0) - Check bitmap for prior receipt</li>
                    <li>Too Old/Beyond Window - Reject as stale duplicate</li>
                    <li>Exact Duplicate (seqDiff = 0) - Reject immediate duplicate</li>
                    <li>Large Forward Gap - Reject as potential desynchronization</li>
                </ol>
            </li>
            <li><strong>Wraparound Handling</strong>: 8-bit sequence numbers wrap at 256 with signed arithmetic boundary adjustment.</li>
            <li><strong>Statistics Tracking</strong>: Maintains counts for original packets, duplicates rejected, and out-of-order packets accepted.</li>
        </ul>
    </div>
    <div style="flex: 1; text-align: right; max-width: 75%;">
        <img src="etherring_pkt_detection_flow.png" alt="Packet Detection Flow" style="width: 110%;">
    </div>
</div>


