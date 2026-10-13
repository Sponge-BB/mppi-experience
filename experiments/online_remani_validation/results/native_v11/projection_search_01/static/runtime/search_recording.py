"""Bounded, read-only V11 search export after command publication (SIL only).

The kernel already retains these candidates for its execution alternatives.
Never enable/disable its trace here: that would change controller behavior.
Binary writing runs off the control thread; a full queue drops telemetry only.
"""
import ctypes as ct
import json
import queue
import threading
import time
import numpy as np


class SearchRecorder:
    def __init__(self, lib, handle, out):
        if not hasattr(lib, 'online_execution_candidates'):
            raise RuntimeError('search recording requires the already tracing V11 kernel')
        self.lib, self.handle = lib, handle
        lib.online_search_trace.argtypes = [ct.c_void_p, ct.POINTER(ct.c_double), ct.c_int]
        lib.online_search_trace.restype = ct.c_int
        self.out = out / 'search'
        self.out.mkdir(exist_ok=False)
        self.queue = queue.Queue(maxsize=2)
        self.next_t = -float('inf')
        self.stats = dict(written=0, dropped=0, oversized=0, errors=[], max_capture_ms=0.)
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def _write(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                # No compression/JSON conversion of thousands of points on the
                # control thread. The queue contains owned numpy copies only.
                np.savez(self.out / ('%06d.npz' % self.stats['written']), **item)
                self.stats['written'] += 1
            except Exception as exc:
                self.stats['errors'].append(str(exc))
            finally:
                self.queue.task_done()

    def capture(self, t, published, state, revision, code, reason, accepted, authorized):
        if t < self.next_t:
            return
        self.next_t = t + .195
        if self.queue.full():
            self.stats['dropped'] += 1
            return
        start = time.monotonic()
        n = self.lib.online_search_trace(self.handle, None, 0) if code == 0 else 0
        if not 0 <= n <= 40000:
            self.stats['oversized'] += 1
            return
        rows = np.empty((n, 8))
        if n:
            got = self.lib.online_search_trace(self.handle, rows.ctypes.data_as(ct.POINTER(ct.c_double)), n)
            if got != n:
                self.stats['errors'].append('trace size changed')
                return
        ms = 1000 * (time.monotonic() - start)
        self.stats['max_capture_ms'] = max(ms, self.stats['max_capture_ms'])
        item = dict(t=t, published=published, state=np.asarray(state).copy() if state is not None else np.empty(0),
                    revision=revision, rows=rows, reason=reason, accepted_candidate=accepted,
                    authorized=authorized, capture_ms=ms)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.stats['dropped'] += 1

    def close(self):
        self.queue.put(None)
        self.worker.join()
        self.stats.update(hz=5, columns=['candidate_id', 'relative_t', 'x', 'y', 'cost', 'gap',
                                       'kernel_feasible', 'kernel_selected'],
                          scope='exported kernel candidates, not every internal sample; selection is NOT execution approval')
        (self.out / 'recording.json').write_text(json.dumps(self.stats, indent=2) + '\n')
