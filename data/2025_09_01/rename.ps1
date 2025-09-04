# Powershell file to rename files based on run number and timestamp
# Usage: Run this script in a Powershell environment

# Mapping of run numbers to timestamps (12-hour format for matching, as in logs.md)
$runMap = @{
    "r1"  = "05_26_29_PM"
    "r2"  = "05_30_22_PM"
    "r3"  = "05_34_09_PM"
    "rXX" = "07_22_20_PM"
    "r4"  = "05_38_17_PM"
    "r5"  = "05_40_57_PM"
    "r6"  = "05_43_36_PM"
    "r7"  = "05_46_45_PM"
    "r8"  = "05_59_02_PM"
    "r9"  = "06_02_01_PM"
    "r10" = "06_04_57_PM"
    "r11" = "06_09_20_PM"
    "r12" = "06_12_24_PM"
    "r13" = "06_15_06_PM"
    "r14" = "06_17_52_PM"
    "r14a"= "06_23_05_PM"
    "r15" = "06_20_34_PM"
    "r16" = "06_26_12_PM"
    "r17" = "06_28_56_PM"
    "r18" = "06_31_54_PM"
    "r19" = "06_34_53_PM"
    "r20" = "06_37_34_PM"
    "r21" = "06_40_24_PM"
    "r22" = "06_43_26_PM"
    "r23" = "06_46_14_PM"
    "r24" = "06_48_58_PM"
    "r25" = "06_51_44_PM"
    "r26" = "06_54_43_PM"
    "r27" = "06_57_24_PM"
    "r28" = "07_00_20_PM"
    "r29" = "07_02_53_PM"
    "r30" = "07_05_40_PM"
    "r31" = "07_08_25_PM"
}

# Directory containing the files
$dir = "d:\git_repos\sumo_cra_traffic_sim\data\2025_09_01\"

# Rename files based on mapping
foreach ($run in $runMap.Keys) {
    $timestamp = $runMap[$run]
    # Find matching files
    $files = Get-ChildItem -Path $dir -Filter "*$timestamp*" | Where-Object { $_.Extension -in ".csv", ".png" }
    foreach ($file in $files) {
        if ($file.Name -notlike "${run}_*") {
            $newName = "${run}_$($file.Name)"
            Rename-Item -Path $file.FullName -NewName $newName
        }
    }
}