#!/bin/bash

#Give system a chance to initialize
sleep 2s

#Check if DAPrototype is running
if pgrep "DAPrototype" > /dev/null
then
	echo "DAPrototype already running!"
else
	#Check for correct display
	DISPLAYONLY=${DISPLAY:0:2}
	if [[ "$DISPLAYONLY" = ":0" ]]
	then
		echo "Launching DAPrototype!" 
		sudo /DAPrototype/build/DAPrototype
	fi
fi
