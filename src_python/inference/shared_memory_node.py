import ctypes as cpp
from multiprocessing.shared_memory import SharedMemory
import numpy as np
import typing
from time import sleep

# notes
# name of shared memory item is telemetryIPC

# sleep(0) in linux just forces the os schedualer to yield the current thread's time slice
# this lets SCHED_FIFO take priority


class TelemetryIPC(cpp.Structure):
    _fields_ = [
        ("sequenceCounter", cpp.c_uint32),
        ("q", cpp.c_double * 3),
        ("qdot", cpp.c_double * 3),
        ("u", cpp.c_double * 3),
        ("estimatedMass", cpp.c_double),
        ("tauPINN", cpp.c_double * 3),
        ("pathVersion", cpp.c_uint32),
    ]


class SharedMemoryIPC:
    shm_block_name: str = "telemetryIPC"
    shm_block_size: int = cpp.sizeof(TelemetryIPC)

    def __init__(self) -> None:
        self.shm = SharedMemory(name=self.shm_block_name, create=False)
        buf = typing.cast(typing.Any, self.shm.buf)
        self.data = TelemetryIPC.from_buffer(buf)

    def read_memory(self) -> tuple[list[float], list[float], list[float], float]:
        while True:
            seqCount1 = self.data.sequenceCounter

            # check if cpp is writting
            if seqCount1 % 2 != 0:
                sleep(0)
                continue

            # copy data
            local_q = list(self.data.q)
            local_qdot = list(self.data.qdot)
            local_u = list(self.data.u)
            local_mass = self.data.estimatedMass

            # Check if the data copied is "good"
            seqCount2 = self.data.sequenceCounter

            if seqCount1 == seqCount2:
                return local_q, local_qdot, local_u, local_mass

            sleep(0)

    def write_memory(self, tau_compensation: list[float]) -> None:
        for i in range(3):
            self.data.tauPINN[i] = tau_compensation[i]
