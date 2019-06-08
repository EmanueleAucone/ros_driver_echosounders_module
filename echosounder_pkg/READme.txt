EXECUTION STEPS:
1) execute the following lines on user laptop shell (sudo if necessary):
	echo "1" > /proc/sys/net/ipv4/ip_forward
	iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
	ssh 192.168.0.90 (connect to the echosouder module)

2) execute the following lines on the payload shell
	sudo route add default gw <ip_own_PC>
	cd /var/lock
	rm LCK..ttyS1
	rm LCK..ttyS3

3) to execute the nodes:
	roslaunch echosounder_pkg test.launch (on the payload shell) -> main that spawn handler and two drivers
	export ROS_MASTER_URI=http://192.168.0.90:11311 (on user shel) -> connect PC to ROS master
	roslaunch echosounder_pkg test_rqt.launch (on user shell) -> launch the GUI (for system status, data logging and high level commands)

4) execute high level command for the handler:
	The GUI contains a specific panel to set the command parameters;
	otherwise, execute the following line on user shell:
	rostopic pub /handler/high_level_command echosounder_pkg/EchoCommand "{echo1: 'echo1', mode1: 'acq', N1: 1, K1: 0.0, echo2: 'echo2', mode2: 'acq', N2: 1, K2: 0.0, repeat: true}" (example of alternate mode)

5) To send another high level command at runtime, execute again step 4

EXTRA:
-If the GUI was enabled to log data, in order to convert from bag file to csv file (offline), execute:
	rostopic echo -b namefile.bag -p topic_name > namefile.csv
	
-To flash the code on the pyaload CPU:
	scp -r path/echosounder_pkg root@192.168.0.90:~/catkin_ws/src


