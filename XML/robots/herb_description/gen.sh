. /opt/ros/kinetic/setup.bash
for input_path in robots/*.xacro
do
	output_path=$(echo $input_path | sed 's/\.xacro//')
	echo $input_path $output_path
	rosrun xacro xacro -o $output_path $input_path
done
