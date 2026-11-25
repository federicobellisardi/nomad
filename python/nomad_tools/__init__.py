"""Pure-Python tools for nomad inputs and outputs (no C++ extension needed).

io            readers for graph.bin, routes.bin and run snapshots
chain_summary per-iteration status of a day-to-day chain (from the simulator logs)
count_compare simulated daily flow vs observed counts on cross-sections
network_load  vehicles on the network by hour
gridlock_scan full links and their strongly connected components (spillback cycles)
launch_chain  generate and submit a day-to-day chain from a city preset

Each analysis module is a CLI: python -m nomad_tools.<module> --help
"""
