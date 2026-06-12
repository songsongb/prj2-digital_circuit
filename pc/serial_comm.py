# serial_comm.py — 통신 레이어 (Mock ↔ 실시리얼 스위치)
#
# 두 모드 모두 "백그라운드 스레드가 줄(line)을 받아 thread-safe 큐에 넣고,
# 메인 렌더 루프는 poll()로 한 번에 비워 처리" 하는 동일한 인터페이스를 갖는다.
# 이렇게 하면 LLM 호출/시리얼 대기가 pygame 루프(60fps)를 막지 않는다.

import threading
import queue
import time

import config


class CommLink:
    """RISC-V(FPGA)와의 양방향 링크 공통 인터페이스."""

    def __init__(self):
        self._rx = queue.Queue()       # 수신한 원시 텍스트 줄
        self._stop = threading.Event()
        self._thread = None

    # ── 공개 API ────────────────────────────────────────────
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def poll(self):
        """수신 큐를 통째로 비워 리스트로 반환 (논블로킹)."""
        lines = []
        while True:
            try:
                lines.append(self._rx.get_nowait())
            except queue.Empty:
                break
        return lines

    def send(self, text):
        """PC → RISC-V 송신. 서브클래스에서 구현."""
        raise NotImplementedError

    def stop(self):
        self._stop.set()

    # ── 내부 ───────────────────────────────────────────────
    def _emit(self, line):
        self._rx.put(line)

    def _run(self):
        raise NotImplementedError


class SerialLink(CommLink):
    """pyserial 기반 실제 UART 링크 (FPGA 연결용)."""

    def __init__(self, port=config.SERIAL_PORT, baud=config.BAUD):
        super().__init__()
        self.port, self.baud = port, baud
        self._ser = None

    def _open(self):
        import serial  # pyserial. 실제 모드에서만 필요
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)

    def _run(self):
        try:
            self._open()
        except Exception as e:
            self._emit(f"AI:[연결 실패] {e}")
            return
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, _, buf = buf.partition(b"\n")
                        self._emit(raw.decode("utf-8", "replace"))
            except Exception:
                time.sleep(0.05)

    def send(self, text):
        if self._ser:
            try:
                self._ser.write(text.encode("utf-8"))
            except Exception:
                pass


class MockLink(CommLink):
    """가짜 시나리오를 시간 흐름대로 흘려보내는 링크 (FPGA 없이 개발/데모)."""

    def __init__(self, scenario):
        super().__init__()
        # scenario(emit, stop_event): 줄을 emit()으로 내보내는 제너레이터/함수
        self._scenario = scenario

    def _run(self):
        self._scenario(self._emit, self._stop)

    def send(self, text):
        # mock에서는 PC→RISC-V 명령을 콘솔로만 확인
        print(f"[MockLink] PC→RISC-V 송신: {text.strip()}")


class EmulatorLink(CommLink):
    """FPGA를 흉내내는 양방향 링크. PC→FPGA 명령(CMD)을 큐로 전달한다."""

    def __init__(self, scenario):
        super().__init__()
        self._scenario = scenario
        self._tx = queue.Queue()          # PC→FPGA 명령

    def _run(self):
        self._scenario(self._emit, self._get_cmd, self._stop)

    def send(self, text):
        self._tx.put(text)

    def _get_cmd(self):
        try:
            return self._tx.get_nowait()
        except queue.Empty:
            return None


def make_link():
    """config.DATA_SOURCE에 따라 적절한 링크 생성 (sim은 링크 없음)."""
    src = getattr(config, "DATA_SOURCE", "sim")
    if src == "serial":
        return SerialLink()
    if src == "emulator":
        from fpga_emulator import emulator_scenario
        return EmulatorLink(emulator_scenario)
    return None
