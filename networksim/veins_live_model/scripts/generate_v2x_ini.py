from pathlib import Path
import os
import yaml

# This script converts a YAML scenario profile into an OMNeT++ ini file.
# The YAML is the human-editable config.
# The generated ini is what OMNeT++ actually reads.

REPO_ROOT = Path(__file__).resolve().parents[3]

CONFIG_PATH = Path(
    os.environ.get(
        "V2X_CONFIG",
        "networksim/veins_live_model/config/attack_dos.yaml",
    )
)

if not CONFIG_PATH.is_absolute():
    CONFIG_PATH = REPO_ROOT / CONFIG_PATH

OUT_PATH = (
    REPO_ROOT
    / "networksim/veins_live_model/simulations/v2x_controller_live/generated/live_generated.ini"
)

with CONFIG_PATH.open("r") as f:
    cfg = yaml.safe_load(f)

scenario = cfg["scenario"]
simu5g = cfg["simu5g"]

# Optional sections with defaults.
v2x_status = cfg.get("v2x_status", {})
dos = cfg.get("dos_attack", {})
outputs = cfg.get("outputs", {})

dos_enabled = bool(dos.get("enabled", False))

status_dest = v2x_status.get("destination", "car[1]")
status_port = int(v2x_status.get("port", 4000))
status_interval = v2x_status.get("send_interval", "0.1s")
status_start = v2x_status.get("start_time", "0.5s")
status_packet_size = int(v2x_status.get("packet_size_bytes", 300))
status_log = outputs.get("status_packet_log", "v2x_simu5g_delivered_packets.csv")

# If DoS is enabled, car[0] and car[1] each need two apps:
# app[0] = real V2X status path
# app[1] = fake-agent DoS/noise path
apps_per_car = 2 if dos_enabled else 1

lines = [
    "# Auto-generated from YAML. Do not edit directly.",
    f"# Source YAML: {CONFIG_PATH}",
    "",
    "[General]",
    f"sim-time-limit = {scenario['sim_time_limit']}",
    "",
    "# Controller bridge",
    "**.controllerBridgeEnabled = true",
    '**.controllerHost = "127.0.0.1"',
    "**.controllerPort = 5555",
    '**.commandLogFile = "v2x_live_controller_commands.csv"',
    "**.useSimu5gDeliveredNv0 = true",
    "",
    "# Simu5G / D2D settings",
    f'**.amcMode = "{simu5g.get("amc_mode", "D2D")}"',
    f"*.car[*].cellularNic.d2dInitialMode = {str(simu5g.get('d2d_initial_mode', True)).lower()}",
    f"*.gNodeB*.cellularNic.nrPhy.enableD2DCqiReporting = {str(simu5g.get('enable_d2d_cqi_reporting', True)).lower()}",
    f"**.usePreconfiguredTxParams = {str(simu5g.get('use_preconfigured_tx_params', False)).lower()}",
]

# Only force d2dCqi if we explicitly choose fixed/preconfigured Tx parameters.
# For the clean Simu5G-base run, use_preconfigured_tx_params should be false.
if bool(simu5g.get("use_preconfigured_tx_params", False)):
    lines.append(f"**.d2dCqi = {simu5g['d2d_cqi']}")

lines += [
    "",
    "# App layout",
    f"*.car[0].numApps = {apps_per_car}",
    f"*.car[1].numApps = {apps_per_car}",
    "",
    "# Real V2X status path: nv0 sends status packets to nv1",
    '*.car[0].app[0].typename = "V2XStatusSenderApp"',
    f'*.car[0].app[0].destAddress = "{status_dest}"',
    f"*.car[0].app[0].localPort = {status_port}",
    f"*.car[0].app[0].destPort = {status_port}",
    f"*.car[0].app[0].sendInterval = {status_interval}",
    f"*.car[0].app[0].startTime = {status_start}",
    f"*.car[0].app[0].packetSizeBytes = {status_packet_size}",
    '*.car[1].app[0].typename = "V2XStatusReceiverApp"',
    f"*.car[1].app[0].localPort = {status_port}",
    f'*.car[1].app[0].packetLogFile = "{status_log}"',
]

if dos_enabled:
    fake_agent_count = int(dos.get("fake_agent_count", 1))
    packets_per_agent_per_second = float(dos.get("packets_per_agent_per_second", 10))
    total_dos_packets_per_second = fake_agent_count * packets_per_agent_per_second

    if total_dos_packets_per_second <= 0:
        raise ValueError("Total DoS packet rate must be greater than zero.")

    dos_send_interval = 1.0 / total_dos_packets_per_second
    dos_dest = dos.get("destination", "car[1]")
    dos_dest_port = int(dos["destination_port"])
    dos_packet_size = int(dos["packet_size_bytes"])
    dos_log = outputs.get("dos_packet_log", "v2x_dos_packets.csv")

    # First DoS test should be channel-load only.
    # So fake traffic should not bind to the same receiver port as real V2X status.
    if dos_dest_port == status_port:
        raise ValueError(
            f"DoS destination_port={dos_dest_port} conflicts with real V2X status port={status_port}. "
            "Use destination_port: 4500 for the first DoS/channel-load test."
        )

    lines += [
        "",
        "# DoS path: fake-agent traffic loads the same Simu5G D2D channel",
        '*.car[0].app[1].typename = "V2XNoiseSenderApp"',
        f'*.car[0].app[1].destAddress = "{dos_dest}"',
        "*.car[0].app[1].localPort = 4500",
        f"*.car[0].app[1].destPort = {dos_dest_port}",
        f"*.car[0].app[1].sendInterval = {dos_send_interval:.9f}s",
        f"*.car[0].app[1].startTime = {dos['attack_start']}",
        f"*.car[0].app[1].stopTime = {dos['attack_stop']}",
        f"*.car[0].app[1].packetSizeBytes = {dos_packet_size}",
        '*.car[1].app[1].typename = "V2XNoiseReceiverApp"',
        f"*.car[1].app[1].localPort = {dos_dest_port}",
        f'*.car[1].app[1].noiseLogFile = "{dos_log}"',
        "",
        "# DoS metadata",
        f"# fake_agent_count = {fake_agent_count}",
        f"# packets_per_agent_per_second = {packets_per_agent_per_second}",
        f"# total_dos_packets_per_second = {total_dos_packets_per_second}",
        f"# impersonate_vehicle_id = {dos.get('impersonate_vehicle_id', 'none')}",
    ]

lines.append("")

OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
OUT_PATH.write_text("\n".join(lines))

print(f"Wrote {OUT_PATH}")
print(f"Using YAML config: {CONFIG_PATH}")

if dos_enabled:
    print("DoS enabled: true")
    print(f"Fake agents: {fake_agent_count}")
    print(f"Total DoS packet rate: {total_dos_packets_per_second} packets/sec")
    print(f"Generated DoS sendInterval: {dos_send_interval:.9f}s")
else:
    print("DoS enabled: false")