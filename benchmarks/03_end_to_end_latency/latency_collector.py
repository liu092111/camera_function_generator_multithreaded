# -*- coding: utf-8 -*-
"""
latency_collector.py — Thread-safe latency data collector for end-to-end
system latency measurement (Method A: software timestamps).
"""

import threading
import csv
import numpy as np


class LatencyCollector:
    """Collects per-frame latency records from the main thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._records = []

    def record(self, frame_id, t_capture, t_queue_in, t_process_done, t_fg_sent):
        """Append one latency record (all times in seconds from perf_counter)."""
        seg1 = (t_queue_in - t_capture) * 1000.0
        seg2 = (t_process_done - t_queue_in) * 1000.0
        seg3 = (t_fg_sent - t_process_done) * 1000.0 if t_fg_sent is not None else float('nan')
        total = (t_fg_sent - t_capture) * 1000.0 if t_fg_sent is not None else float('nan')

        with self._lock:
            self._records.append({
                'frame_id': frame_id,
                't_capture': t_capture,
                't_queue_in': t_queue_in,
                't_process_done': t_process_done,
                't_fg_sent': t_fg_sent if t_fg_sent is not None else float('nan'),
                'seg1': seg1,
                'seg2': seg2,
                'seg3': seg3,
                'total': total,
            })

    def count(self):
        with self._lock:
            return len(self._records)

    def get_stats(self):
        """Return per-segment statistics (ms).  NaN frames excluded from seg3/total."""
        with self._lock:
            recs = list(self._records)

        if not recs:
            return None

        seg1 = np.array([r['seg1'] for r in recs])
        seg2 = np.array([r['seg2'] for r in recs])
        seg3 = np.array([r['seg3'] for r in recs])
        total = np.array([r['total'] for r in recs])

        seg3_valid = seg3[np.isfinite(seg3)]
        total_valid = total[np.isfinite(total)]

        def _s(arr):
            if len(arr) == 0:
                return {k: float('nan') for k in ['mean', 'std', 'min', 'max', 'p50', 'p95', 'p99']}
            return {
                'mean': float(np.mean(arr)),
                'std': float(np.std(arr)),
                'min': float(np.min(arr)),
                'max': float(np.max(arr)),
                'p50': float(np.percentile(arr, 50)),
                'p95': float(np.percentile(arr, 95)),
                'p99': float(np.percentile(arr, 99)),
            }

        return {
            'n_total': len(recs),
            'n_with_fg': len(total_valid),
            'seg1': _s(seg1),
            'seg2': _s(seg2),
            'seg3': _s(seg3_valid),
            'total': _s(total_valid),
        }

    def save_csv(self, path):
        """Write raw per-frame records to CSV."""
        with self._lock:
            recs = list(self._records)

        cols = ['frame_id', 't_capture', 't_queue_in', 't_process_done',
                't_fg_sent', 'seg1', 'seg2', 'seg3', 'total']

        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for r in recs:
                w.writerow(r)
