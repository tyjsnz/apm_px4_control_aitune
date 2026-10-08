# -*- coding: utf-8 -*-
"""任务线程内的 print/stdout 路由.

飞控控制函数(apm_control/apm_control_test)大量用 print 输出进度, 服务端用线程局部
变量把每个任务的输出收集到该任务自己的日志缓冲, 同时可转发到服务控制台.
"""
import sys
import threading

_local = threading.local()


class TaskLogRouter:
    def __init__(self, orig=None, mirror=True):
        self.orig = orig if orig is not None else sys.__stdout__
        self.mirror = mirror
        self._lock = threading.Lock()

    def install(self):
        sys.stdout = self
        sys.stderr = self
        return self

    @property
    def sink(self):
        return getattr(_local, 'sink', None)

    def write(self, text):
        sink = self.sink
        if sink is not None and text:
            sink(text)
        if self.mirror and self.orig is not None:
            try:
                self.orig.write(text)
            except Exception:
                pass
        return len(text)

    def flush(self):
        if self.mirror and self.orig is not None:
            try:
                self.orig.flush()
            except Exception:
                pass


class TaskLogBuffer:
    """单个任务的输出缓冲 (只在任务线程内使用)."""

    def __init__(self, max_lines=4000):
        self.max_lines = max_lines
        self.lines = []
        self.partial = ''
        self.lock = threading.Lock()

    def __call__(self, text):
        with self.lock:
            self.partial += text
            while '\n' in self.partial:
                line, self.partial = self.partial.split('\n', 1)
                self.lines.append(line)
                if len(self.lines) > self.max_lines:
                    del self.lines[:len(self.lines) - self.max_lines]

    def snapshot(self, tail=None):
        with self.lock:
            data = list(self.lines)
            if self.partial:
                data.append(self.partial)
        if tail:
            return data[-tail:]
        return data


def bind(sink):
    _local.sink = sink
    return sink


def unbind():
    _local.sink = None
