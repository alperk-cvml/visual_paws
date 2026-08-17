#!/bin/bash
# Increase max UDP receive/send buffer sizes
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=2147483647

# Increase IP fragmentation thresholds for large point cloud payloads
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.ipv4.ipfrag_low_thresh=134217728

