#!/usr/bin/env bash
set -euo pipefail

# CSV and PNG files whose timestamps differ by no more than this value
# are considered part of the same run.
PAIR_WINDOW_SECONDS=5

# By default, only show the proposed renames.
# Run with --apply to perform them.
APPLY=false

if [[ ${1:-} == "--apply" ]]; then
    APPLY=true
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--apply]" >&2
    exit 1
fi

tmp_file=$(mktemp)
trap 'rm -f "$tmp_file"' EXIT

shopt -s nullglob

files=(
    sumIndoorVIL_log_*.csv
    sumIndoorVIL_log_*.png
)

if (( ${#files[@]} == 0 )); then
    echo "No matching CSV or PNG files found."
    exit 0
fi

# Extract each filename's timestamp and convert it to Unix time.
for file in "${files[@]}"; do
    filename=${file##*/}

    # Skip files that have already received a run number.
    if [[ $filename =~ _r[0-9]+\.(csv|png)$ ]]; then
        continue
    fi

    if [[ $filename =~ ^sumIndoorVIL_log_([0-9]{4})_([0-9]{2})_([0-9]{2})-([0-9]{2})_([0-9]{2})_([0-9]{2})_(AM|PM)\.(csv|png)$ ]]; then
        year=${BASH_REMATCH[1]}
        month=${BASH_REMATCH[2]}
        day=${BASH_REMATCH[3]}
        hour=${BASH_REMATCH[4]}
        minute=${BASH_REMATCH[5]}
        second=${BASH_REMATCH[6]}
        am_pm=${BASH_REMATCH[7]}

        timestamp="$year-$month-$day $hour:$minute:$second $am_pm"

        if ! epoch=$(LC_ALL=C date -d "$timestamp" +%s 2>/dev/null); then
            echo "Could not parse timestamp in: $filename" >&2
            exit 1
        fi

        printf '%s\t%s\n' "$epoch" "$file" >> "$tmp_file"
    else
        echo "Skipping unrecognized filename: $filename" >&2
    fi
done

if [[ ! -s $tmp_file ]]; then
    echo "No unnumbered matching files found."
    exit 0
fi

mapfile -t sorted_files < <(sort -n -k1,1 "$tmp_file")

declare -a sources=()
declare -a destinations=()
declare -A cluster_extensions=()

run_number=-1
cluster_start=-1

for entry in "${sorted_files[@]}"; do
    epoch=${entry%%$'\t'*}
    file=${entry#*$'\t'}

    if (( cluster_start < 0 || epoch - cluster_start > PAIR_WINDOW_SECONDS )); then
        run_number=$((run_number + 1))
        cluster_start=$epoch
    fi

    extension=${file##*.}
    stem=${file%.*}
    destination="${stem}_r${run_number}.${extension}"

    key="${run_number}:${extension}"
    if [[ -n ${cluster_extensions[$key]+x} ]]; then
        echo "Warning: run r${run_number} contains multiple .$extension files." >&2
        echo "Consider reducing PAIR_WINDOW_SECONDS." >&2
    fi
    cluster_extensions[$key]=1

    sources+=("$file")
    destinations+=("$destination")
done

# Check every destination before renaming anything.
for destination in "${destinations[@]}"; do
    if [[ -e $destination ]]; then
        echo "Refusing to overwrite existing file: $destination" >&2
        exit 1
    fi
done

for i in "${!sources[@]}"; do
    source=${sources[$i]}
    destination=${destinations[$i]}

    printf '%s -> %s\n' "$source" "$destination"

    if [[ $APPLY == true ]]; then
        mv -- "$source" "$destination"
    fi
done

if [[ $APPLY == false ]]; then
    echo
    echo "Dry run only. Run with --apply to perform these renames."
fi
