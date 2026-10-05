"""Automated parking in Project Chrono. Start with agent.py (the state machine) or cli.py (the entry point)."""

import os

# NumPy's OpenBLAS starts a thread per core for the small matrix products of the image processing
# and keeps them spinning between calls. That gains nothing for one simulation and makes four of
# them on one machine run several times slower than one after the other. It also makes the last
# digits of a least-squares fit depend on the number of cores. This has to be set before NumPy loads.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
