# Powershell file to rename files based on run number and timestamp
# Usage: Run this script in a Powershell environment

# Mapping of run numbers to timestamps (24-hour format for matching)
$runMap = @{
    "r1"  = "02_06_26_PM"
    "r2"  = "02_11_09_PM"
    "r3"  = "02_15_19_PM"
    "r4"  = "02_20_27_PM"
    "r5"  = "02_24_09_PM"
    "r6"  = "02_28_44_PM"
    "r7"  = "02_35_41_PM"
    "r8"  = "02_38_42_PM"
    "r9"  = "02_41_53_PM"
    "r10" = "02_45_34_PM"
    "r11" = "02_48_47_PM"
    "r12" = "02_51_44_PM"
    "r13" = "02_54_39_PM"
    "r14" = "02_58_45_PM"
    "r15" = "03_04_24_PM"
    "r16" = "03_24_43_PM"
    "r17" = "03_39_13_PM"
    "r18" = "03_44_52_PM"
    "r19a" = "03_50_36_PM" # route 1
    "r19b" = "03_53_09_PM" # route 2
    "r20" = "03_56_17_PM"
    "r21" = "03_59_23_PM"
    "r22" = "04_02_54_PM"
    "r23" = "04_05_24_PM"
    "r24" = "04_21_15_PM"
    "r25" = "04_25_19_PM"
    "r26" = "04_28_01_PM"
    "r27" = "04_30_25_PM"
    "r28" = "04_34_59_PM"
    "r29" = "04_37_50_PM"
    "r30" = "04_42_17_PM"
    "r31" = "04_45_10_PM"
}

# Directory containing the files
$dir = "d:\git_repos\sumo_cra_traffic_sim\data\2025_08_31\"

# Rename files based on mapping
foreach ($run in $runMap.Keys) {
    $timestamp = $runMap[$run]
    # Find matching files
    $files = Get-ChildItem -Path $dir -Filter "*$timestamp*" | Where-Object { $_.Extension -in ".csv", ".png" }
    foreach ($file in $files) {
        $newName = "${run}_$($file.Name)"
        Rename-Item -Path $file.FullName -NewName $newName
    }
}