#!/bin/bash

log_folder="./log"
completed_file="$log_folder/completed"
log_file="$log_folder/log"

if [ ! -d "$log_folder" ]; then
  mkdir "$log_folder"
fi

touch "$log_file"
touch "$completed_file"

# use a route_id list for the for looop

route_ids=(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15)

for route_id in "${route_ids[@]}"; do
  for weather in {0..11}; do
    # Skip route if it has already been completed
    if grep -q -x "${route_id}_${weather}$" $completed_file; then
      echo "Skipping Route: $route_id, Weather: $weather (already completed)"
    else
      # Start Carla it is not already running
      carla_pid=$(pgrep -f "CarlaUE4.sh")
      if [ -z "$carla_pid" ]; then
        echo "Starting Carla..."
        cd ~/carla && ./CarlaUE4.sh > /dev/null 2>&1 &
      fi

      # Wait for Carla to fully load
      echo "Waiting for Carla to load..."
      while ! nc -z localhost 2000; do
        sleep 1
      done

      # Start generating route dataset
      echo "Generating Dataset for Route: $route_id, Weather: $weather"
      timeout 500 python EI_Drive.py -t record --route_id $route_id --weather $weather >"${log_folder}/route_${route_id}_weather_${weather}" 2>&1
      exit_status=$?

      if [ $exit_status -eq 124 ]; then
        message="Python program timed out after 5 minutes for Route: $route_id, Weather: $weather. Exiting..."
        echo $message
        echo $message >> $log_file
      elif [ $exit_status -ne 0 ]; then
        message="Python program terminated unexpectedly for Route: $route_id, Weather: $weather. Exiting..."
        echo $message
        echo $message >> $log_file
      else
        message="Generated Dataset for Route: $route_id, Weather: $weather"
        echo $message
        echo "${route_id}_${weather}" >> $completed_file
        echo $message >> $log_file
      fi
    fi
  done
done
