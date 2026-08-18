#!/usr/bin/env python3
import os
import pty
import select
import signal
import subprocess
import sys
import time


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(payload):
    crc = crc16_modbus(payload)
    return bytes(payload) + bytes((crc & 0xFF, crc >> 8))


def read_exact(fd, size, deadline):
    result = bytearray()
    while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f"timeout after {bytes(result).hex(' ')}")
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            continue
        chunk = os.read(fd, size - len(result))
        if not chunk:
            raise RuntimeError("pseudo terminal closed")
        result.extend(chunk)
    return bytes(result)


def require_lines(output, expected_lines):
    lines = output.splitlines()
    for line in expected_lines:
        if line not in lines:
            raise RuntimeError(f"missing {line!r}:\n{output}")


def common_execute_arguments(device, extra):
    return [
        "--board", "front",
        *extra,
        "--device", device,
        "--timeout-ms", "500",
        "--execute",
        "--confirm-wheels-off-ground",
        "--confirm-hardware-stop-ready",
        "--confirm-exclusive-tty",
    ]


MODE_REQUEST = with_crc([0x01, 0x03, 0x00, 0x08, 0x00, 0x01])
FULL_STATUS_REQUEST = with_crc([0x01, 0x03, 0x00, 0x00, 0x00, 0x0C])
MODE_RESPONSE = with_crc([0x01, 0x03, 0x02, 0x00, 0x01])
FULL_STATUS_RESPONSE = with_crc([
    0x01, 0x03, 0x18,
    0x01, 0xE1, 0x00, 0x00,
    0x00, 0x00, 0xFC, 0x55,
    0x00, 0x00, 0x3A, 0x5B,
    0x01, 0xF4, 0x00, 0x64,
    0x00, 0x01, 0x00, 0xEE,
    0x00, 0x00, 0x01, 0x0F,
])


def full_status_response_with(register_index, value):
    payload = bytearray(FULL_STATUS_RESPONSE[:-2])
    offset = 3 + register_index * 2
    payload[offset] = (value >> 8) & 0xFF
    payload[offset + 1] = value & 0xFF
    return with_crc(payload)


ALARM_STATUS_RESPONSE = full_status_response_with(10, 0x0005)
COAST_FRAME = with_crc([0x01, 0x10, 0x00, 0x00, 0x00, 0x00])
M1_FORWARD_FRAME = with_crc([0x01, 0x10, 0x01, 0xE1, 0x00, 0x00])
INITIALIZATION_FRAMES = [
    with_crc([0x01, 0x06, 0x00, 0x09, 0x00, 0x01]),
    with_crc([0x01, 0x06, 0x00, 0x07, 0x00, 0x01]),
    with_crc([0x01, 0x06, 0x00, 0x0F, 0x00, 0x01]),
    with_crc([0x01, 0x06, 0x00, 0x05, 0x00, 0x00]),
    with_crc([0x01, 0x06, 0x00, 0x06, 0x00, 0x00]),
]


def start_pty_process(executable, extra):
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    process = subprocess.Popen(
        [executable] + common_execute_arguments(device, extra),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return process, master_fd, slave_fd


def answer_read_preflight(master_fd):
    for expected, response in [(MODE_REQUEST, MODE_RESPONSE)]:
        actual = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if actual != expected:
            raise RuntimeError(
                f"unexpected preflight request {actual.hex(' ')}, "
                f"expected {expected.hex(' ')}"
            )
        os.write(master_fd, response)


def answer_jog_preflight(master_fd, status_response=None):
    answer_read_preflight(master_fd)
    request = read_exact(master_fd, 8, time.monotonic() + 2.0)
    if request != FULL_STATUS_REQUEST:
        raise RuntimeError(
            f"unexpected jog preflight request: {request.hex(' ')}"
        )
    os.write(
        master_fd,
        FULL_STATUS_RESPONSE if status_response is None else status_response,
    )


def close_process(process, master_fd, slave_fd):
    os.close(master_fd)
    os.close(slave_fd)
    if process.poll() is None:
        process.kill()
        process.wait()


def test_dry_run(production_executable):
    process = subprocess.run(
        [
            production_executable,
            "--board", "rear",
            "--motor", "M2",
            "--direction", "reverse",
            "--rpm", "100",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if process.returncode != 0:
        raise RuntimeError(f"dry-run failed ({process.returncode}):\n{process.stdout}")
    expected_jog = with_crc([0x01, 0x10, 0x00, 0x00, 0x06, 0x42])
    require_lines(
        process.stdout,
        [
            "protocol_source=MOTOR_REGISTER_TABLE_V4",
            "board=rear",
            "connector=JP18",
            "expected_device=/dev/ttyS1",
            "motor=M2",
            "direction=table_reverse_state_2",
            "rpm=100",
            "duration_ms=300",
            "planned_jog=" + " ".join(f"{byte:02X}" for byte in expected_jog),
            "mode=DRY_RUN",
            "serial_opened=NO",
            "write_operations=NOT_EXECUTED",
        ],
    )


def test_50_rpm_company_table_frame(production_executable):
    process = subprocess.run(
        [
            production_executable,
            "--board", "front",
            "--motor", "M1",
            "--direction", "forward",
            "--rpm", "50",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    expected = with_crc([0x01, 0x10, 0x03, 0x21, 0x00, 0x00])
    if process.returncode != 0:
        raise RuntimeError(f"50 rpm dry-run failed:\n{process.stdout}")
    require_lines(
        process.stdout,
        [
            "rpm=50",
            "planned_jog=" + " ".join(f"{byte:02X}" for byte in expected),
        ],
    )


def test_device_mismatch_blocked(production_executable):
    process = subprocess.run(
        [
            production_executable,
            *common_execute_arguments(
                "/dev/ttyS1", ["--motor", "M1", "--direction", "forward"]
            ),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if process.returncode != 3:
        raise RuntimeError(
            f"device mismatch returned {process.returncode}, expected 3:\n"
            f"{process.stdout}"
        )
    require_lines(
        process.stdout,
        [
            "result=BLOCKED",
            "reason=DEVICE_DOES_NOT_MATCH_BOARD_PROFILE",
            "serial_opened=NO",
            "write_operations=NOT_EXECUTED",
        ],
    )


def test_safety_confirmations_required(test_executable):
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    try:
        process = subprocess.run(
            [
                test_executable,
                "--board", "front",
                "--motor", "M1",
                "--direction", "forward",
                "--device", device,
                "--execute",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    finally:
        os.close(master_fd)
        os.close(slave_fd)
    if process.returncode != 3:
        raise RuntimeError(
            f"missing confirmations returned {process.returncode}, expected 3:\n"
            f"{process.stdout}"
        )
    require_lines(
        process.stdout,
        [
            "result=BLOCKED",
            "reason=SAFETY_CONFIRMATIONS_INCOMPLETE",
            "serial_opened=NO",
            "write_operations=NOT_EXECUTED",
        ],
    )


def test_initialize_dry_run(production_executable):
    process = subprocess.run(
        [production_executable, "--board", "front", "--initialize"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if process.returncode != 0:
        raise RuntimeError(
            f"initialize dry-run failed ({process.returncode}):\n{process.stdout}"
        )
    require_lines(
        process.stdout,
        [
            "operation=REGISTER_TABLE_V4_INITIALIZE",
            "function=0x06_WRITE_SINGLE_NO_ACK_READ",
            "initialization_order=EXISTING_FIRMWARE_SEQUENCE",
            "response_handling=NO_ACK_READ_MATCH_EXISTING_FIRMWARE",
            "planned_pre_coast=" + " ".join(f"{byte:02X}" for byte in COAST_FRAME),
            "planned_init_clear_alarm="
            + " ".join(f"{byte:02X}" for byte in INITIALIZATION_FRAMES[0]),
            "planned_init_clear_hall="
            + " ".join(f"{byte:02X}" for byte in INITIALIZATION_FRAMES[1]),
            "planned_init_set_follow="
            + " ".join(f"{byte:02X}" for byte in INITIALIZATION_FRAMES[2]),
            "planned_init_m1_decel_zero="
            + " ".join(f"{byte:02X}" for byte in INITIALIZATION_FRAMES[3]),
            "planned_init_m2_decel_zero="
            + " ".join(f"{byte:02X}" for byte in INITIALIZATION_FRAMES[4]),
            "planned_final_coast="
            + " ".join(f"{byte:02X}" for byte in COAST_FRAME),
            "mode=DRY_RUN",
            "serial_opened=NO",
            "write_operations=NOT_EXECUTED",
        ],
    )


def test_coast_only(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--coast-only"]
    )
    try:
        answer_read_preflight(master_fd)
        actual = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if actual != COAST_FRAME:
            raise RuntimeError(f"unexpected Coast frame: {actual.hex(' ')}")
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if process.returncode != 0:
        raise RuntimeError(f"Coast-only failed ({process.returncode}):\n{output}")
    require_lines(
        output,
        ["operation=COAST_BOTH_MOTORS", "coast_write=OK", "result=OK"],
    )


def test_status_only(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--status-only"]
    )
    try:
        answer_read_preflight(master_fd)
        state_request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if state_request != FULL_STATUS_REQUEST:
            raise RuntimeError(
                f"unexpected status request: {state_request.hex(' ')}"
            )
        os.write(master_fd, FULL_STATUS_RESPONSE)
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if process.returncode != 0:
        raise RuntimeError(f"status-only failed ({process.returncode}):\n{output}")
    require_lines(
        output,
        [
            "operation=READ_SMALL_CAR_STATUS_ONLY",
            "function=0x03_READ_ONLY",
            "mode_raw=0x0001",
            "m1_state_raw=0x01E1",
            "m1_state_rpm=30",
            "m1_state_code=1",
            "m1_state=FORWARD",
            "m2_state_raw=0x0000",
            "m2_state_rpm=0",
            "m2_state_code=0",
            "m2_state=FREE_STOP",
            "m1_hall_count_raw=0x0000FC55",
            "m1_hall_count=64597",
            "m2_hall_count_raw=0x00003A5B",
            "m2_hall_count=14939",
            "m1_feedback_rpm=500",
            "m2_feedback_rpm=100",
            "status_mode_raw=0x0001",
            "bus_voltage_raw=0x00EE",
            "bus_voltage_mv=23800",
            "alarm_code_raw=0x0000",
            "program_version_raw=0x010F",
            "write_operations=NOT_EXECUTED",
            "result=OK",
        ],
    )
    if "planned_coast=" in output or "planned_jog=" in output:
        raise RuntimeError(f"status-only advertised a write plan:\n{output}")


def test_jog_sequence(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd)
        pre_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog_observed_at = time.monotonic()
        observed = [pre_coast, jog]
        state_request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if state_request != FULL_STATUS_REQUEST:
            raise RuntimeError(
                f"unexpected state request: {state_request.hex(' ')}"
            )
        os.write(master_fd, FULL_STATUS_RESPONSE)
        observed.append(read_exact(master_fd, 8, time.monotonic() + 2.0))
        coast_observed_at = time.monotonic()
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if observed != [COAST_FRAME, M1_FORWARD_FRAME, COAST_FRAME]:
        raise RuntimeError(
            "unexpected jog sequence: " + ", ".join(x.hex(" ") for x in observed)
        )
    jog_duration = coast_observed_at - jog_observed_at
    if not 0.20 <= jog_duration <= 0.40:
        raise RuntimeError(
            f"jog duration {jog_duration:.3f}s outside safe test tolerance"
        )
    if process.returncode != 0:
        raise RuntimeError(f"jog failed ({process.returncode}):\n{output}")
    require_lines(
        output,
        [
            "mode_raw=0x0001",
            "pre_coast_write=OK",
            "jog_write=OK",
            "m1_state_raw=0x01E1",
            "m2_state_raw=0x0000",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "result=OK",
        ],
    )


def test_preflight_alarm_blocks_all_writes(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd, ALARM_STATUS_RESPONSE)
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if process.returncode != 5:
        raise RuntimeError(
            f"preflight alarm returned {process.returncode}, expected 5:\n{output}"
        )
    require_lines(
        output,
        [
            "alarm_code_raw=0x0005",
            "result=BLOCKED",
            "reason=PREFLIGHT_STATUS_NOT_SAFE",
            "write_operations=NOT_EXECUTED",
        ],
    )


def test_active_alarm_coasts_and_fails(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd)
        pre_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog = read_exact(master_fd, 8, time.monotonic() + 2.0)
        request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if request != FULL_STATUS_REQUEST:
            raise RuntimeError(f"unexpected active request: {request.hex(' ')}")
        os.write(master_fd, ALARM_STATUS_RESPONSE)
        final_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if pre_coast != COAST_FRAME or jog != M1_FORWARD_FRAME:
        raise RuntimeError("unexpected frames before active alarm")
    if final_coast != COAST_FRAME:
        raise RuntimeError("active alarm did not end with Coast")
    if process.returncode != 6:
        raise RuntimeError(
            f"active alarm returned {process.returncode}, expected 6:\n{output}"
        )
    require_lines(
        output,
        [
            "alarm_code_raw=0x0005",
            "validation=UNSAFE_STATUS",
            "final_coast_result=OK",
            "result=ERROR",
        ],
    )


def test_active_status_timeout_coasts_by_deadline(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd)
        pre_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog_observed_at = time.monotonic()
        status_request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        final_coast = read_exact(master_fd, 8, time.monotonic() + 1.0)
        coast_observed_at = time.monotonic()
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)

    if pre_coast != COAST_FRAME or jog != M1_FORWARD_FRAME:
        raise RuntimeError("unexpected sequence before active-status timeout")
    if status_request != FULL_STATUS_REQUEST:
        raise RuntimeError(
            f"unexpected active-status request: {status_request.hex(' ')}"
        )
    if final_coast != COAST_FRAME:
        raise RuntimeError("active-status timeout did not end with Coast")
    jog_duration = coast_observed_at - jog_observed_at
    if not 0.20 <= jog_duration <= 0.40:
        raise RuntimeError(
            f"timeout Coast arrived after {jog_duration:.3f}s, expected about 0.300s"
        )
    if process.returncode != 6:
        raise RuntimeError(
            f"active-status timeout returned {process.returncode}, expected 6:\n{output}"
        )
    require_lines(
        output,
        [
            "stage=active_status",
            "transaction_error=transaction timeout",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "result=ERROR",
        ],
    )


def test_interrupt_during_active_status_coasts_immediately(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd)
        pre_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog = read_exact(master_fd, 8, time.monotonic() + 2.0)
        status_request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        signal_sent_at = time.monotonic()
        process.send_signal(signal.SIGINT)
        final_coast = read_exact(master_fd, 8, time.monotonic() + 1.0)
        coast_observed_at = time.monotonic()
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)

    if pre_coast != COAST_FRAME or jog != M1_FORWARD_FRAME:
        raise RuntimeError("unexpected sequence before active-status SIGINT")
    if status_request != FULL_STATUS_REQUEST:
        raise RuntimeError(
            f"unexpected active-status request: {status_request.hex(' ')}"
        )
    if final_coast != COAST_FRAME:
        raise RuntimeError("active-status SIGINT did not end with Coast")
    interrupt_latency = coast_observed_at - signal_sent_at
    if interrupt_latency > 0.15:
        raise RuntimeError(
            f"SIGINT Coast latency {interrupt_latency:.3f}s exceeded 0.150s"
        )
    if process.returncode != 130:
        raise RuntimeError(
            f"active-status SIGINT returned {process.returncode}, expected 130:\n{output}"
        )
    require_lines(
        output,
        [
            "stage=active_status",
            "transaction_error=transport error",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "result=ERROR",
        ],
    )


def test_initialize_sequence(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--initialize"]
    )
    try:
        answer_read_preflight(master_fd)
        observed = [
            read_exact(master_fd, 8, time.monotonic() + 2.0)
            for _ in range(7)
        ]
        post_mode_request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if post_mode_request != MODE_REQUEST:
            raise RuntimeError(
                f"unexpected post-initialize mode request: "
                f"{post_mode_request.hex(' ')}"
            )
        os.write(master_fd, MODE_RESPONSE)
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    expected = [COAST_FRAME, *INITIALIZATION_FRAMES, COAST_FRAME]
    if observed != expected:
        raise RuntimeError(
            "unexpected initialize sequence: "
            + ", ".join(frame.hex(" ") for frame in observed)
        )
    if process.returncode != 0:
        raise RuntimeError(
            f"initialize failed ({process.returncode}):\n{output}"
        )
    require_lines(
        output,
        [
            "pre_coast_write=OK",
            "init_clear_alarm_write=OK",
            "init_clear_hall_write=OK",
            "init_set_follow_write=OK",
            "init_m1_decel_zero_write=OK",
            "init_m2_decel_zero_write=OK",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "post_init_mode_raw=0x0001",
            "result=OK",
        ],
    )


def test_initialize_preflight_timeout(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--initialize"]
    )
    try:
        request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if request != MODE_REQUEST:
            raise RuntimeError(
                f"unexpected initialize preflight request: {request.hex(' ')}"
            )
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if process.returncode != 5:
        raise RuntimeError(
            f"initialize timeout returned {process.returncode}, expected 5:\n{output}"
        )
    require_lines(
        output,
        [
            "stage=mode",
            "transaction_error=transaction timeout",
            "write_operations=NOT_EXECUTED",
            "result=ERROR",
        ],
    )


def test_initialize_interrupt_coasts(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--initialize"]
    )
    try:
        request = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if request != MODE_REQUEST:
            raise RuntimeError(
                f"unexpected initialize preflight request: {request.hex(' ')}"
            )
        process.send_signal(signal.SIGINT)
        os.write(master_fd, MODE_RESPONSE)
        final_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if final_coast != COAST_FRAME:
        raise RuntimeError("initialize preflight SIGINT did not end with Coast")
    if process.returncode != 130:
        raise RuntimeError(
            f"initialize interrupt returned {process.returncode}, expected 130:\n"
            f"{output}"
        )
    require_lines(
        output,
        [
            "stop_requested=YES",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "result=ERROR",
        ],
    )


def test_interrupt_coasts(test_executable):
    process, master_fd, slave_fd = start_pty_process(
        test_executable, ["--motor", "M1", "--direction", "forward"]
    )
    try:
        answer_jog_preflight(master_fd)
        pre_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        jog = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if pre_coast != COAST_FRAME or jog != M1_FORWARD_FRAME:
            raise RuntimeError("unexpected sequence before SIGINT")
        process.send_signal(signal.SIGINT)
        final_coast = read_exact(master_fd, 8, time.monotonic() + 2.0)
        if final_coast != COAST_FRAME:
            raise RuntimeError("SIGINT did not end with Coast")
        output, _ = process.communicate(timeout=2.0)
    finally:
        close_process(process, master_fd, slave_fd)
    if process.returncode != 130:
        raise RuntimeError(
            f"interrupt returned {process.returncode}, expected 130:\n{output}"
        )
    require_lines(
        output,
        [
            "stop_requested=YES",
            "final_coast_write=OK",
            "final_coast_result=OK",
            "result=ERROR",
        ],
    )


def main():
    production_executable = sys.argv[1]
    test_executable = sys.argv[2]
    test_dry_run(production_executable)
    test_50_rpm_company_table_frame(production_executable)
    test_device_mismatch_blocked(production_executable)
    test_safety_confirmations_required(test_executable)
    test_initialize_dry_run(production_executable)
    test_status_only(test_executable)
    test_coast_only(test_executable)
    test_jog_sequence(test_executable)
    test_preflight_alarm_blocks_all_writes(test_executable)
    test_active_alarm_coasts_and_fails(test_executable)
    test_active_status_timeout_coasts_by_deadline(test_executable)
    test_interrupt_during_active_status_coasts_immediately(test_executable)
    test_interrupt_coasts(test_executable)
    test_initialize_sequence(test_executable)
    test_initialize_preflight_timeout(test_executable)
    test_initialize_interrupt_coasts(test_executable)


if __name__ == "__main__":
    main()
