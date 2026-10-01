"""One wall-clock renderer; wakeups coalesce instead of queuing sensor work."""
import threading
import time


class VisualizationWorker:
    def __init__(self, render, period, on_error, name='web-visualization'):
        self.render, self.period, self.on_error = render, period, on_error
        self.condition = threading.Condition()
        self.pending = False
        self.closed = False
        self.wakeups_coalesced = 0
        self.thread = threading.Thread(target=self.run, name=name, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def wake(self):
        with self.condition:
            if self.pending:
                self.wakeups_coalesced += 1
            self.pending = True
            self.condition.notify()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.thread.join(timeout=2)

    def run(self):
        deadline = time.monotonic()
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending,
                    timeout=max(0, deadline - time.monotonic()))
                if self.closed:
                    return
                self.pending = False
            started = time.monotonic()
            try:
                self.render()
            except Exception as exc:
                self.on_error(exc)
            # Expensive work cannot accumulate overdue ticks. Only the latest
            # sensor references are used on the next iteration.
            deadline = max(started + max(.05, self.period()), time.monotonic())
