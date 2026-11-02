"""Exercise serial ownership and supervisor cleanup using a virtual serial port."""
import csv
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import serial


RECORDER = Path(__file__).with_name('plot_joystick.py')


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.output = Path(self.folder.name) / 'samples.csv'
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.master)
        self.addCleanup(os.close, self.slave)
        self.port = os.ttyname(self.slave)

    def command(self, parent=None):
        return [sys.executable, str(RECORDER), '--port', self.port, '--no-plot',
                '--output', str(self.output), '--parent-pid',
                str(os.getpid() if parent is None else parent)]

    def test_records_real_serial_frames_and_closes_cleanly(self):
        process = subprocess.Popen(self.command(), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        self.addCleanup(self.stop, process)
        deadline = time.monotonic() + 5.
        while not self.output.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(self.output.exists(), 'Recorder did not open the virtual port')
        os.write(self.master, b'10,2043,1987,0\n20,2000,1900,1\n30,2200,2100,0\n')
        rows = []
        while time.monotonic() < deadline:
            with self.output.open() as stream:
                rows = list(csv.DictReader(stream))
            if len(rows) == 3:
                break
            time.sleep(.01)
        self.assertEqual([int(row['x_raw']) for row in rows], [2043, 2000, 2200])
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5.)
        self.assertEqual(process.returncode, 0, output)
        with serial.Serial(self.port, exclusive=True):
            pass

    def test_competing_reader_fails_before_recording(self):
        with serial.Serial(self.port, exclusive=True):
            result = subprocess.run(self.command(), capture_output=True, text=True, timeout=5.)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('exclusively lock', result.stderr)
        self.assertFalse(self.output.exists())

    def test_missing_supervisor_releases_the_port(self):
        result = subprocess.run(self.command(parent=1), capture_output=True, text=True, timeout=5.)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Block runner exited', result.stdout)
        with serial.Serial(self.port, exclusive=True):
            pass

    @staticmethod
    def stop(process):
        if process.poll() is None:
            process.kill()
        process.communicate()


if __name__ == '__main__':
    unittest.main()
