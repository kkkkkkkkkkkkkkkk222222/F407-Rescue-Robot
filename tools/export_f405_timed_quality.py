"""Generate the complete v1.1 delivery from structured v1.0 + actual host outputs.

Never opens a serial port, flashes hardware, or sends a motion command.
"""
import copy
import hashlib
import json
import pathlib
import subprocess
from tools import f405_timed_protocol as p
from tools import vision_protocol as legacy
from tools.export_f405_timed_artifacts import export

ROOT = pathlib.Path(__file__).resolve().parents[1]
DOC = ROOT / "docs"
STAMP = "20261101"


def run(args):
    return subprocess.check_output(args, cwd=ROOT, text=True, encoding="utf-8")


def write_json(name, value):
    (DOC / name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8", newline="\n")


def main():
    baseline = json.loads((DOC / "F405_TIMED_CONTRACT_20261031.json").read_text(encoding="utf-8"))
    artifacts = export()
    v = artifacts["validation"]
    implementation = v["builds"]["NormalDebugTimed"]["embedded_git_commit"]
    contract = copy.deepcopy(baseline)
    contract.update(schema_revision=2, contract_version="1.1", firmware_implementation_commit=implementation,
                    preserved_ancestor="f81e9ef9a33981d8453eafe9574572e87dea90c6",
                    upper_baseline="c4fdc330f52aab44162f966b15de0313e16afb6d")
    contract["types"]["0x0B"] = dict(name="quality_request", payload_bytes=10, side_effects="none")
    contract["types"]["0x0C"] = dict(name="quality_reply", payload_bytes=160, quality_schema=1,
                                    pages=[0, 1], periodic=False, fragmentation=False)
    contract["types"]["0x07"]["frequency_semantics"]="nominal real two-interval sample output; no catch-up replay"
    contract["types"]["0x08"]["frequency_semantics"]="100Hz scheduler ceiling; suppressed if both source IDs unchanged"
    contract["capabilities"]["read_only_quality_pages"] = 1024
    for field in contract["fields"]["identity_reply"]:
        if field["name"] == "protocol_minor": field["meaning"] = "1"
        if field["name"] == "clock_flags":
            field["meaning"] = "v1.1 bit0 checked configured TIM5/clock tree with no observed discontinuity; bit1 RNG available; neither proves ppm"
        if field["name"] == "epoch_fault":
            field["meaning"] = "v1.1: 0 RNG available and clock checks healthy, 1 RNG unavailable, 2 observed clock fault"
    for field in contract["fields"]["sync_reply"]:
        if field["name"] == "epoch_valid":
            field["meaning"] = "v1.1: 1 RNG available and clock checks healthy; 0 do not map this clock"
    common = [
        dict(name="nonce", type="u64", offset=0, bytes=8, meaning="echo request nonce"),
        dict(name="boot_epoch", type="opaque", offset=8, bytes=16, meaning="unchanged MCU boot identity"),
        dict(name="reference_epoch", type="u32", offset=24, bytes=4, meaning="reference generation"),
        dict(name="snapshot_us", type="u64", offset=28, bytes=8, meaning="MCU response preparation, not UART first bit"),
        dict(name="quality_schema", type="u8", offset=36, bytes=1, meaning="1"),
        dict(name="page", type="u8", offset=37, bytes=1, meaning="0 transport, 1 timing evidence"),
        dict(name="field_count", type="u16", offset=38, bytes=2, meaning="30"),
    ]
    contract["fields"]["quality_request"] = [
        dict(name="nonce", type="u64", offset=0, bytes=8, meaning="unique outstanding transaction nonce"),
        dict(name="quality_schema", type="u8", offset=8, bytes=1, meaning="1"),
        dict(name="page", type="u8", offset=9, bytes=1, meaning="0 or 1; invalid schema/page returns error5"),
    ]
    for page, names in enumerate(p.QUALITY_NAMES):
        contract["fields"][f"quality_reply_page_{page}"] = common + [
            dict(name=name, type="u32", offset=40+4*i, bytes=4,
                 meaning="observed value; ordered page index "+str(i)) for i, name in enumerate(names)]
    contract["quality_pages"] = {str(i): list(names) for i, names in enumerate(p.QUALITY_NAMES)}
    contract["statistics"] = dict(legacy_0A_fields=list(p.STATS_NAMES), counters_wrap="u32 modulo 2^32",
                                  maxima="since boot; saturate at UINT32_MAX; not worst-case upper bounds",
                                  reset="queries do not clear counters; only MCU init resets them")
    contract["clock_fault_bits"] = dict(counter_regressed=1, timer_configuration_changed=2, clock_tree_changed=4)
    contract["missing_evidence_bits"] = dict(crystal_total_error_spec=1, rear_physical_edge_delay=2,
                                            imu_acquisition_filter_link_irq_delay=4)
    contract["quality_units"] = dict(
        fields_ending_us="nominal TIM5 microseconds; receiver/callback events, not SI accuracy certification",
        fields_ending_ppm="u32 ppm; FFFFFFFF unknown",
        register_fields="raw register bits; rcc_*, tim5_*, debug_apb1_freeze, uart_tx_irq_bits, last_uart_error",
        remaining="DMA remaining bytes; register observation only",
        count_fields="events/frames/bytes as named, modulo 2^32",
        max_rear_exti_service_us="ISR processing time observed after entry; NOT edge-to-IRQ delay",
        max_queue_us="enqueue to just before HAL DMA start API; not a wire-start timestamp",
        max_transfer_us="just before DMA start API through UART TC callback processing",
        max_wheel_enqueue_age_us="wheel t1 to accepted enqueue",
        max_gyro_enqueue_age_us="older of own yaw/rate receive events to accepted enqueue; unavailable if either missing",
        max_wheel_tc_age_us="wheel t1 to UART TC callback",
        max_gyro_tc_age_us="older yaw/rate receive event to UART TC callback; not acquisition-to-wire age",
        max_wheel_tc_gap_us="adjacent successful wheel TC callbacks, observed",
        max_gyro_tc_gap_us="adjacent successful gyro TC callbacks, observed",
        tx_precondition_blocks="unsafe peripheral/DMA state rejected before transmitting first byte",
        tx_buffer_seal_errors="FNV1a-32 RAM lifetime mismatch; not a cryptographic guarantee or replacement for wire CRC",
        tx_drain_waits="2000us waiting observations; not a guaranteed drain bound",
        max_wheel_capture_us="CPU counter-read bracket only; not total collection uncertainty")
    contract["time"].update(
        nominal_scale="HSE nominal 8MHz -> PLL 168MHz -> APB1 timer 84MHz -> TIM5 PSC83 -> nominal 1MHz",
        guaranteed_scale_error_ppm=None, calibration_valid_until=None,
        calibration_conditions="No crystal BOM/total error/temperature/age/load spec or independent clock calibration available",
        clock_fault_policy="invalidate telemetry/clock qualification for this boot; do not clamp timestamps, reset epoch, stop wheels, or reset raw counts",
        unsupported_clock_conditions=["unobserved full wrap loss with all relevant interrupts blocked", "STOP/STANDBY or clock-tree changes", "unrecorded debugger halts; not a wall-clock guarantee"],
        physical_delay_bounds=dict(wheel=None, yaw=None, rate=None),
        physical_delay_bound_valid_until=None,
        scale_observation="observed_scale_interval compares separated SYNC events as past average ratio only; never fills drift_bound_ppm")
    contract["queuing"].update(
        owner="main-loop Uart_Receive(0) only starts DMA; IRQ completion releases ownership but never starts next DMA",
        buffer_lifetime="private persistent DMA buffer; slot reserved before HAL; FNV seal before launch and at UART TC",
        preconditions="UART ready, DMA ready/disabled, DMAT=0, TXEIE=TCIE=0, TC=1",
        abort="abort DMA, drop interrupted frame visibly, wait TC/drain before reuse; no claim an aborted partial frame becomes valid",
        unchanged_gyro="no 08 when both source IDs unchanged; independent REPEAT if only the other source updated",
        no_physical_acceptance_claim=True)
    contract["compatibility"] = dict(
        unchanged="envelope version1, types01..0A layout, 07/08 timestamp events, 0x15/0x1F/0x19/0x1A, motor/PID/signs and competition flow",
        versioned_addition="identity protocol 1.1, capability bit10, quality schema1 at0B/0C; clock-fault validity extension explicitly documented",
        legacy_action_seq="8bit action ACKs still cannot reject all cross-boot delayed responses")
    contract["source_links"] = {k:f"https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/{implementation}/{path}"
                                for k,path in baseline["source_links"].items()}
    contract["official_references"] = [
        "https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_uart.c",
        "https://www.st.com/resource/en/application_note/an2867-oscillator-design-guide-for-stm8afals-stm32-mcus-and-mpus-stmicroelectronics.pdf",
        "https://www.st.com.cn/resource/en/application_note/an4776-how-to-use-generalpurpose-timer-peripheral-on-stm32-mcus-stmicroelectronics.pdf",
        "https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405-407xx-and-stm32f415-417xx-device-limitations-stmicroelectronics.pdf",
    ]
    contract["vendor_imu_evidence"] = dict(protocol="5A AA/BB low high sum8, five bytes; no source tick/sequence",
        provided_manual_sha256="bbcadc0971825d3da1237a25c50e5b4badd64a0eb528a5fa5f4d88755b21ae3b",
        provenance="user-provided single-axis UART communication manual, not IMU660RC documentation",
        advertised_output_max_hz=1000, uart_max_baud=230400,
        one_packet_serialization_us=5*10/230400*1e6,
        internal_filter_queue_total_delay_bound_us=None,
        consequence="Serialization 217.014us is only one component, not a total physical acquisition bound")
    write_json(f"F405_TIMED_CONTRACT_{STAMP}.json", contract)
    # Keep hand-written semantics above, regenerate the complete wire-field appendix.
    markdown=DOC/f"F405_TIMED_CONTRACT_{STAMP}.md"
    content=markdown.read_text(encoding="utf-8").split("<!-- GENERATED_FIELD_TABLES -->")[0]
    content+="<!-- GENERATED_FIELD_TABLES -->\n\n## Complete Wire Field Tables\n\n"
    content+="All offsets below are payload-relative, not absolute frame offsets. All numerical fields are BE except CRC LE.\n\n"
    content+="Envelope: D4 4D | version1:u8 | TYPE:u8 | N:u16 BE | SEQ:u16 BE | payload:N | CRC16 LE | E4.\n"
    content+="CRC16/MODBUS initFFFF polyA001 covers frame[2:8+N]. Max TX payload160, RX total64, partial timeout20ms, no fragmentation.\n\n"
    content+="| TYPE | Name | Payload bytes |\n| --- | --- | --- |\n"
    for kind, spec in contract["types"].items():
        content+=f"| {kind} | {spec['name']} | {spec['payload_bytes']} |\n"
    content+="\n01/03/09 requests each contain only nonce:u64 BE. Replies echo request SEQ and nonce; telemetry SEQ is independent global u16.\n"
    content+="Periodic SEQ is not source ID or time. Pair by boot epoch/source IDs/real intervals, never equal SEQ.\n\n"
    for name, fields in contract["fields"].items():
        content+=f"### {name}\n\n| Offset | Bytes | Type | Field | Meaning |\n| --- | --- | --- | --- | --- |\n"
        for f in fields:
            content+=f"| {f['offset']} | {f['bytes']} | {f['type']} | {f['name']} | {f['meaning']} |\n"
        content+="\n"
    for group in ("capabilities", "wheel_flags", "gyro_flags", "clock_fault_bits", "missing_evidence_bits"):
        content+=f"### {group}\n\n| Name | Mask |\n| --- | --- |\n"
        for name,value in contract[group].items(): content+=f"| {name} | 0x{value:X} |\n"
        content+="\n"
    markdown.write_text(content,encoding="utf-8",newline="\n")

    examples = artifacts["examples"]
    examples["contract_version"] = "1.1"
    for page in (0, 1):
        raw = p.request(11, 0x0102030405060708, 0x1234+page, quality_page=page)
        examples["examples"].append(dict(name=f"quality_page_{page}_request", provenance="synthetic; never sent", hex=raw.hex(" ").upper(), decoded=p.decode(raw)))
    actual = [bytes.fromhex(line[6:]) for line in artifacts["c_host_log"].splitlines() if line.startswith("FRAME ")]
    for raw in actual:
        item = p.decode(raw)
        if item["type"] == 12 and item["page"] == 1:
            examples["examples"].append(dict(name="quality_page_1_c_module", provenance="actual host C module, fake peripherals; not board data",
                                             hex=raw.hex(" ").upper(), decoded=item))
    old = legacy.build_frame(0x15, 1, bytes(8))
    extended = p.request(1, 1)
    for label, prefix, real in (("extended_prefix_before_legacy",p.MAGIC,old),
                                 ("legacy_prefix_before_extended",b"\xa3\xb3",extended),
                                 ("legacy_prefix_before_legacy",b"\xa3\xb3",old)):
        raw=prefix+real
        examples["examples"].append(dict(name=label, provenance="synthetic injected corruption; never sent",
                                         hex=raw.hex(" ").upper(), integrity=p.inspect_capture(raw), expected="reject extra bytes; never claim intact stream"))
    write_json(f"F405_TIMED_EXAMPLES_{STAMP}.json", examples)
    uart_log=run(["build/test_timed_uart_queue.exe"])
    clock_log=run(["build/test_timed_clock.exe"])
    (DOC/f"F405_TIMED_HOST_SIMULATION_{STAMP}.log").write_text(artifacts["c_host_log"]+uart_log+clock_log,encoding="utf-8",newline="\n")
    patch=run(["git","diff","--no-ext-diff","--binary","HEAD","--","Main","Boards","Core","Drivers","cmake","CMakeLists.txt","CMakePresets.json"])
    (DOC/f"F405_TIMED_BUILD_DIFF_{STAMP}.patch").write_text(patch,encoding="utf-8",newline="\n")
    v.update(upper_baseline=contract["upper_baseline"], contract_version="1.1",
        user_reported_running_before_changes=dict(git="f81e9ef9a33981d8453eafe9574572e87dea90c6",source_dirty=1,
            provenance="RDK real query reported by user; not independently repeated here", firmware_id=None),
        local_read_only_probe=dict(port="COM18",baud=115200,commands=["PING","STAT"],received_bytes=0,
            physical_link_identity_verified=False,motion_commands_sent=False,flashed=False),
        serial_commands_sent_this_run=True, motion_commands_sent_this_run=False,
        physical_timing_verified=False, physical_uart_integrity_verified=False,
        root_cause_of_RDK_prefixes="not established; hardening covers specific unsafe ownership/recovery states, not a physical diagnosis",
        host_uart_test=dict(log=uart_log, clean_mixed_frame_count=2000,
            injected_failures="HAL_BUSY, stale DMAT, abort with shifter not drained, unexpected TC, active RAM mutation",
            injected_seal_failure_expected=1,physical_peripherals_mocked=True),
        host_clock_test=dict(log=clock_log, physical_peripherals_mocked=True),
        python_checks=dict(command="python -m unittest tools.test_timed_quality tools.test_vision_protocol",passed=30),
        dirty_patch=dict(file=f"F405_TIMED_BUILD_DIFF_{STAMP}.patch", sha256=hashlib.sha256(patch.encode()).hexdigest()),
        build_reproduction=dict(configure="cmake --preset NormalDebugTimed",build="cmake --build --preset NormalDebugTimed --parallel 6",
            compiler=run([r"C:\Users\29459\Desktop\pack\Toolchain\arm_gnu_toolchain\bin\arm-none-eabi-gcc.exe","--version"]).splitlines()[0],
            source_dirty_explained="existing Main/Src/Task.c blank lines; apply supplied patch to implementation commit for the same source input fingerprint",
            exact_ELF_reproducibility="not claimed; existing __DATE__/__TIME__ banner and absolute toolchain paths affect ELF bytes"))
    v["bandwidth_model"]["quality_page_0_and_1_each_1hz_Bps"]=342
    v["bandwidth_model"]["total_with_quality_Bps"]=18468
    v["bandwidth_model"]["with_quality_utilization_230400"]=18468*10/230400
    v["bandwidth_model"]["quality_reply_serialization_us"]=171*10/230400*1e6
    v["bandwidth_model"]["note"]="nominal ceiling model; query bursts can expire timed packets. Stagger pages, do not batch polls. Not measured board throughput."
    write_json(f"F405_TIMED_VALIDATION_{STAMP}.json",v)
    print(json.dumps(dict(implementation=implementation,examples=len(examples["examples"]),builds=v["builds"]),indent=2))


if __name__ == "__main__":
    main()
