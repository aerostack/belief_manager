#!/bin/bash
file=$(readlink -f "$0")
filepath=$(dirname "$file")
roslaunch belief_manager_process belief_manager_process.launch config_path:="$filepath" config_file:="belief_manager_config_1.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res != *ERROR* ]]
then
	echo "Test if the belief_manager_config.yaml is correct: succeeded"
else 
	echo "Test if the belief_manager_config.yaml is correct: failed"
fi
rm temp.txt
kill -2 %1
roslaunch belief_manager_process belief_manager_process.launch config_path:="$filepath" config_file:="nothing.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res == *"YAMLEXCEPTION! CALLER_FUNCTION: YAML FILE ERROR"* ]]
then
	echo "Test if the belief_manager_config.yaml doesn't exist: succeeded"
else 
	echo "Test if the belief_manager_config.yaml doesn't exist: failed"	
fi
rm temp.txt
kill -2 %1
roslaunch belief_manager_process belief_manager_process.launch config_path:="$filepath" config_file:="belsief_manager_config_2.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res == *"YAMLEXCEPTION! CALLER_FUNCTION: YAML FILE ERROR"* ]]
then
	echo "Test if there is not a value for a label: succeeded"
else 
	echo "Test if there is not a value for a label: failed"	
fi
rm temp.txt
kill -2 %1
roslaunch belief_manager_process belief_manager_process.launch config_path:="$filepath" config_file:="belief_manager_config_3.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res == *"YAMLEXCEPTION! CALLER_FUNCTION: YAML FILE ERROR"* ]]
then
	echo "Test 1 if a label is incorrect: succeeded"
else 
	echo "Test 1 if a label is incorrect: failed"	
fi
rm temp.txt
kill -2 %1
roslaunch belief_manager_process belief_manager_process.launch config_path:="$filepath" config_file:="belief_manager_config_4.yaml" > /dev/null 2> temp.txt &
echo "Waiting 3 seconds..."
sleep 3
res=$(cat temp.txt)
res=${res^^}
if [[ $res == *"ERROR: EMERGENCY_EVENTS NOT FOUND IN BELIEF_MANAGER_CONFIG"* ]]
then
	echo "Test 2 if a label is incorrect: succeeded"
else 
	echo "Test 2 if a label is incorrect: failed"	
fi
rm temp.txt
kill -2 %1














