#!/bin/bash

i="0"
while [ $i -lt 2 ]
do
	pkill -9 python

	clear

	python3 -W ignore main.py --is_sim --obj_mesh_dir 'objects/actual_objects/rc_v2' --push_rewards --experience_replay --explore_rate_decay --save_visualizations
	python3 -W ignore main.py --is_sim --push_rewards --experience_replay --explore_rate_decay --save_visualizations

	folder=$(<logs_latestFolder.txt)
	#savefile="evaluation_$i.txt"
	savefile="evaluation_ur3_$i.txt"

	python3 evaluate.py --session_directory 'logs/'$folder --method 'reinforcement' --num_obj_complete 10 > $savefile
	#python3 evaluate.py --session_directory 'logs/'$folder --method 'reinforcement' --num_obj_complete 10 > evaluation.txt
	i=$[$i+1]
done
#shutdown +2
