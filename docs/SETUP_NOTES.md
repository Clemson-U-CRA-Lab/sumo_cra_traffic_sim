# SUMO Simulation Run Notes

Basic steps to run the SUMO-only simulation after the WSL/SUMO setup has already been completed.

## Run Instructions

1. Open WSL from PowerShell:

```powershell
wsl
```

2. Navigate to the repo:

```bash
cd ~/projects/sumo_cra_traffic_sim
```

3. Activate the Python virtual environment:

```bash
source .venv/bin/activate
```

4. Run the simulation script:

```bash
python scripts/sumoOnly_v3.py
```

## Checks / Notes

* Make sure you are on the correct branch:

```bash
git branch
```

Expected branch:

```bash
cohda_v2x_networksim
```

If needed, switch branches:

```bash
git checkout cohda_v2x_networksim
```

* Make sure `SUMO_HOME` is set:

```bash
echo $SUMO_HOME
```

Expected output:

```bash
/usr/share/sumo
```

If `SUMO_HOME` is not set, run:

```bash
echo 'export SUMO_HOME="/usr/share/sumo"' >> ~/.bashrc
source ~/.bashrc
```

