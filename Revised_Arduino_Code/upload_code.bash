#!/bin/bash

# Check if the required input is provided
if [ -z "$1" ]; then
  echo "Error: No input provided."
  echo "Usage: $0 <input> [com_port]"
  exit 1
fi

# Assign the inputs to variables
INPUT=$1
COM_PORT=$2

# Construct the command
COMMAND="pio run -t upload -e ROLLER_$INPUT"

# Add the optional com port if provided
if [ ! -z "$COM_PORT" ]; then
  COMMAND="$COMMAND --upload-port $COM_PORT"
fi

# Execute the command
echo "Running: $COMMAND"
$COMMAND
