"""Print reproducible offline fixtures/validation as JSON. Never opens hardware."""
import hashlib
import json
import math
import pathlib
import re
import struct
import subprocess
from tools import f405_timed_protocol as p
from tools import vision_protocol as legacy


def export():
    epoch = bytes.fromhex("00112233445566778899AABBCCDDEEFF")
    epoch2 = bytes.fromhex("102132435465768798A9BACBDCEDFE0F")
    examples = []

    def add(name, raw, provenance="synthetic encoding, never sent", **extra):
        try:
            decoded = p.decode(raw)
        except ValueError as error:
            decoded = {"expected_rejection": str(error)}
        examples.append(dict(name=name, provenance=provenance, hex=raw.hex(" ").upper(), decoded=decoded, **extra))

    def wheel(mid, t0, t1, delta, raw=None, flags=129, ep=epoch):
        raw = [(1000+x) & 0xFFFFFFFF for x in delta] if raw is None else raw
        k = math.pi*.070/1061.268
        vx = round(k*(delta[0]-delta[1])/(math.sqrt(3)*(t1-t0)*1e-6)*1e6) if t1 > t0 else -2147483648
        vy = round(k*(delta[0]+delta[1]-2*delta[2])/(3*(t1-t0)*1e-6)*1e6) if t1 > t0 else -2147483648
        if not flags & 1:
            vx = vy = -2147483648
        payload = struct.pack(">16sIIQQiiIIIiiiIII", ep, 1, mid, t0, t1, vx, vy,
                              *raw, *delta, flags, 0xFFFFFFFF, 2)
        return p.packet(7, mid & 65535, payload)

    def gyro(mid, yaw, rate, flags=835, pub=1020000):
        return p.packet(8, mid & 65535, struct.pack(">16sIIqQIIIiQIIIQ", epoch, 1, mid,
            yaw, 1019000, 100, flags, 0xFFFFFFFF, rate, 1019500, 101, flags, 0xFFFFFFFF, pub))

    for kind, name in [(1,"identity_request"),(3,"sync_request"),(5,"enable_stream_request"),(9,"stats_request")]:
        add(name, p.request(kind, 0x0102030405060708, 0x1234, enable=True))
    for name, delta in [("forward",[20,-20,0]),("reverse",[-20,20,0]),("left",[20,20,-40]),
                        ("right",[-20,-20,40]),("stationary_valid_feedback",[0,0,0])]:
        add(name, wheel(100,1000000,1020000,delta), sdk_eligible=False,
            reason="source uncertainty unknown; slip not independently verified")
    add("invalid_feedback_not_zero", wheel(101,1000000,1020000,[0,0,0],flags=128|512))
    add("gyro_ccw", gyro(200,90000,30000))
    add("gyro_cw", gyro(201,-90000,-30000))
    add("gyro_stationary", gyro(202,90000,0))
    add("gyro_old_value_resend", gyro(203,90000,0,flags=899,pub=1025000))
    add("gyro_stale_resend", gyro(204,90000,0,flags=914,pub=1060000))
    add("low32_tick_wrap_in_u64_clock", wheel(0xFFFFFFFE,(1<<32)-10000,(1<<32)+10000,[20,-20,0]))
    add("raw_u32_counter_wrap", wheel(0xFFFFFFFF,1000000,1020000,[20,-20,0],raw=[4,0xFFFFFFF0,0]))
    add("measurement_id_wrap_to_zero", wheel(0,1020000,1040000,[20,-20,0]))
    add("new_mcu_boot", wheel(1,10000,30000,[20,-20,0],ep=epoch2),must_reset_clock_mapping=True)
    first = wheel(301,2000000,2020000,[20,-20,0],raw=[20,0xFFFFFFEC,0])
    third = wheel(303,2040000,2060000,[20,-20,0],raw=[60,0xFFFFFFC4,0])
    add("missing_packet_before",first,missing_measurement_ids=[302])
    add("missing_packet_after",third,missing_measurement_ids=[302],
        note="raw difference spans missing intervals; delta remains this packet's own 20ms interval")
    batch = (legacy.build_frame(0x15,7,bytes.fromhex("D4 4D 00 00 00 00 0A 07")) + first
             + gyro(305,90000,30000) + legacy.imu_frame(8,90000,300,1,3))
    add("batch_arrival_mixed_legacy",batch,extracted_extended_count=len(p.stream_frames(batch)),
        all_arrival_times_may_be_equal=True,note="host batch arrival is not source time")
    bad = bytearray(p.request(1,8)); bad[-3] ^= 1
    add("bad_crc_request",bytes(bad))
    add("unsupported_version_request",p.packet(1,1,(8).to_bytes(8,"big"),version=2))
    result = subprocess.run(["build/test_timed_telemetry.exe"],capture_output=True,text=True,check=True)
    actual = [bytes.fromhex(line[6:]) for line in result.stdout.splitlines() if line.startswith("FRAME ")]
    seen = set()
    for raw in actual:
        key = p.decode(raw)["type"]
        if key not in seen:
            add("c_module_host_fixture_type_"+str(key),raw,
                "actual C module output in x86 host simulation, fake clock/epoch/UID; not board data")
            seen.add(key)
    source = subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
    builds = {}
    for name in ["NormalDebugTimed","NormalDebug","NormalDebugTask","MotionDebug"]:
        elf = pathlib.Path("build")/name/"WWW.elf"
        item = dict(compiled=elf.exists(),flashed_this_run=False,
                    elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest() if elf.exists() else None)
        header = elf.parent/"generated/DebugBuildIdentity.h"
        if header.exists():
            text = header.read_text()
            item["embedded_firmware_id"] = re.search(r'DEBUG_FIRMWARE_ID_HEX "([^"]+)"',text).group(1)
            values = re.search(r'DEBUG_GIT_ID_BYTES \{([^}]+)',text).group(1).replace(","," ").split()
            item["embedded_git_commit"] = bytes(int(x,16) for x in values).hex()
            item["source_dirty"] = int(re.search(r'DEBUG_SOURCE_DIRTY (\d+)',text).group(1))
        builds[name] = item
    rates = dict(legacy_odom=1500,legacy_imu=1500,motion_status=300,new_wheel=4750,new_gyro=9500,
                 sync_5hz=285,identity_1hz=148,stats_1hz=143)
    total = sum(rates.values())
    compiled_commit=builds["NormalDebugTimed"].get("embedded_git_commit",source)
    validation = dict(implementation_commit=compiled_commit,checkout_commit=source,
        upper_baseline="278370a",board_verified_this_run=False,
        flashed_this_run=False,serial_commands_sent_this_run=False,
        host_c_module_test=dict(passed=True,compiler="GCC x86_64 -Wall -Wextra -Werror",
            decoded_actual_c_frames=len(actual),hardware_platform_mocked=True,physical_timing_verified=False,
            output_summary=result.stdout.splitlines()[-1]),
        legacy_protocol_checks=dict(passed=23,scope="existing Python checks, not physical/control acceptance"),
        builds=builds,bandwidth_model=dict(classification="capacity calculation, NOT measured UART throughput/latency",
            bytes_per_second=rates,total_Bps=total,utilization_115200=total*10/115200,
            utilization_230400=total*10/230400,profile_enable_minimum_baud=230400,
            source_frame_serialization_us_at_230400=95*10/230400*1e6,max_unsent_timed_queue_age_us=8000),
        board_latency_log=None,board_clock_drift_bound_ppm=None,imu_acquisition_to_receive_bound_us=None,
        rear_exti_edge_to_counter_bound_us=None,sdk_gate=dict(qualified=False,
            reason="unknown source-time bounds and clock drift; no new-board performance evidence"),
        stats_fields=list(p.STATS_NAMES))
    return dict(examples=dict(implementation_commit=compiled_commit,wire_version=1,all_robot_packets_sent=False,
                             examples=examples),validation=validation,c_host_log=result.stdout)


if __name__ == "__main__":
    print(json.dumps(export(),ensure_ascii=False))
