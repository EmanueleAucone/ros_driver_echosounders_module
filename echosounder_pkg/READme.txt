STEP DI ESECUZIONE:
1) eseguire le seguenti operazioni sul proprio PC (con sudo se necessario):
	echo "1" > /proc/sys/net/ipv4/ip_forward
	iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
	ssh 192.168.0.90 (password: pippolo)

2) eseguire le seguenti operazioni sul payload
	sudo route add default gw <ip_proprio_PC>
	cd /var/lock
	rm LCK..ttyS1
	rm LCK..ttyS3

3) lanciare i comandi per far partire i nodi:
	roslaunch echosounder_pkg test.launch (sul payload) -> main che lancia handler e i 2 driver
	export ROS_MASTER_URI=http://192.168.0.90:11311 (sul proprio PC) -> connette il PC al ros_master
	roslaunch echosounder_pkg test_rqt.launch (sul proprio PC) -> fa partire la GUI, che contiene anche la parte per il log

4) lanciare il comando di alto livello per l'handler:
	Dalla GUI si può inviare il comando mediante il pannello predisposto
	oppure eseguire il comando sul proprio PC:
	rostopic pub /handler/high_level_command echosounder_pkg/EchoCommand "{echo1: 'echo1', mode1: 'acq', N1: 1, K1: 0.0, echo2: 'echo2', mode2: 'acq', N2: 1, K2: 0.0, repeat: true}" 

4) per inviare un nuovo comando di alto livello a runtime, eseguire nuovamente lo step 4

NOTE INERENTI LA GUI:
-La GUI è formata da 2 plot sulla sinistra, 1 su cui sono riportati slope e altitudine (switch è sotto il grafico) e l'altro dove ci sono i range misurati dai due echosounder (switch sotto)
-Nella parte in alto a destra della GUI è presente un pannello per il settaggio e l'invio dei parametri del comando di alto livello
-Nella parte in basso a destra della GUI è presente la sezione per il log, che inizia quando si clicca sul pulsante rosso (record)
-Se è stato avviato il log, per convertire da bag file a csv file (off-line), lanciare:
	rostopic echo -b namefile.bag -p topic_name > namefile.csv
	(al posto di topic_name indicare /driver1/info/range oppure /driver2/info/range oppure /handler/sea_bottom, come riportato nella medesima sezione della GUI)

EXTRA:
-Per flashare il codice sulla CPU del payload
	scp -r percorso/echosounder_pkg root@192.168.0.90:~/catkin_ws/src

Per una visione più ampia si rimanda all'Appendice del report

