#!/usr/bin/env python3
"""
OSPI PHY Grapher Data Plotter - Simple visualization

This script reads the binary PHY tuning data and creates a simple scatter plot
showing the passing points for each rdDelay value in different colors.

Data Format:
- 81,920 bytes total
- Organization: [5 rdDelay values][128 txDLL values][128 rxDLL values]
- Each value is 1 byte (0 = fail, >0 = pass)

Usage:
    python3 ospi_phy_grapher_plotter.py data.bin [rx_dll tx_dll]

    Optional: Provide reference points to plot (e.g., old/new tuning points)
"""

import struct
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


class OspiPhyGrapherPlotter:
    """OSPI PHY Grapher simple data visualization."""

    # Data organization
    NUM_RD_DELAYS = 5
    NUM_TX_DLL = 128
    NUM_RX_DLL = 128
    BYTES_PER_HEATMAP = NUM_TX_DLL * NUM_RX_DLL  # 16384 bytes
    TOTAL_BYTES = NUM_RD_DELAYS * BYTES_PER_HEATMAP  # 81920 bytes

    # Colors for different rdDelay values
    COLORS = ['green', 'blue', 'red', 'orange', 'purple']

    def __init__(self, data_file):
        """Initialize plotter with binary data file."""
        self.data_file = Path(data_file)
        self.heatmaps = []
        self.load_data()

    def load_data(self):
        """Load and parse binary data file."""
        try:
            with open(self.data_file, 'rb') as f:
                data = f.read()

            # Parse into 5 heatmaps
            for rd_delay in range(self.NUM_RD_DELAYS):
                offset = rd_delay * self.BYTES_PER_HEATMAP
                heatmap_data = data[offset:offset + self.BYTES_PER_HEATMAP]

                # Convert to 2D array (128×128)
                heatmap = np.frombuffer(heatmap_data, dtype=np.uint8)
                heatmap = heatmap.reshape((self.NUM_TX_DLL, self.NUM_RX_DLL))

                self.heatmaps.append(heatmap)

        except FileNotFoundError:
            print(f"[ERROR] File not found: {self.data_file}")
            sys.exit(1)
        except Exception as e:
            print(f"[ERROR] Failed to load data: {e}")
            sys.exit(1)

    def plot_passing_points(self, ref_points=None):
        """
        Plot passing points for all rdDelay values on a single plot.

        Args:
            ref_points: List of reference points to mark (e.g., tuning algorithm results)
                       Format: [(rx, tx), (rx, tx), ...]
        """
        plt.figure(figsize=(12, 10))

        # Plot passing points for each rdDelay with different color
        has_legend_items = False
        for rd_delay in range(self.NUM_RD_DELAYS):
            heatmap = self.heatmaps[rd_delay]

            # Collect passing coordinates only
            pass_x, pass_y = [], []

            for tx in range(self.NUM_TX_DLL):
                for rx in range(self.NUM_RX_DLL):
                    if heatmap[tx, rx] > 0:  # Only passing points
                        pass_x.append(tx)  # TX on X-axis
                        pass_y.append(rx)  # RX on Y-axis

            # Plot passing points
            if pass_x:
                has_legend_items = True
                pass_rate = len(pass_x) / (self.NUM_TX_DLL * self.NUM_RX_DLL) * 100
                plt.scatter(pass_x, pass_y, c=self.COLORS[rd_delay], s=20, alpha=0.7,
                           label=f'rdDelay={rd_delay} ({len(pass_x)} points)')

        # Plot reference points if provided
        if ref_points:
            for i, (tx, rx) in enumerate(ref_points):
                if i == 0:
                    plt.plot(tx, rx, color='black', marker='o', markersize=10,
                            linestyle='dashed', label='Old Tuning Algorithm Point')
                    has_legend_items = True
                elif i == 1:
                    plt.plot(tx, rx, color='magenta', marker='*', markersize=15,
                            linestyle='dashed', label='New Tuning Algorithm Point')
                    has_legend_items = True

        # Labels and formatting
        plt.xlim(0, self.NUM_TX_DLL)
        plt.ylim(0, self.NUM_RX_DLL)
        plt.xlabel('TX DLL Value', fontsize=12, fontweight='bold')
        plt.ylabel('RX DLL Value', fontsize=12, fontweight='bold')
        plt.title('OSPI PHY Tuning - Passing Points by rdDelay', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)

        # Only show legend if there are items to display
        if has_legend_items:
            plt.legend(loc='upper right', fontsize=10)

        plt.show()


def main():
    """Command-line interface for plotter."""
    if len(sys.argv) < 2:
        print("Usage: python3 ospi_phy_grapher_plotter.py <data_file> [tx1 rx1 tx2 rx2]")
        print("\nExample:")
        print("  python3 ospi_phy_grapher_plotter.py data.bin")
        print("  python3 ospi_phy_grapher_plotter.py data.bin 50 60 60 70")
        print("\nReference points: [tx rx] pairs for old and new tuning algorithm results")
        sys.exit(1)

    data_file = sys.argv[1]

    # Create plotter
    plotter = OspiPhyGrapherPlotter(data_file)

    # Parse reference points if provided (up to 2 points: tx1 rx1 tx2 rx2)
    ref_points = None
    if len(sys.argv) >= 4:
        try:
            tx1 = int(sys.argv[2])
            rx1 = int(sys.argv[3])
            ref_points = [(tx1, rx1)]

            # Second point (optional)
            if len(sys.argv) >= 6:
                tx2 = int(sys.argv[4])
                rx2 = int(sys.argv[5])
                ref_points.append((tx2, rx2))
        except (ValueError, IndexError):
            pass

    # Generate visualization
    plotter.plot_passing_points(ref_points)


if __name__ == '__main__':
    main()
