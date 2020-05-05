# ESP32_Camera_alarm_system
ESP32 Camera server, remotely controllable via HTTP and via UDP, with movement detection, failover mode, 
ability to send captured pictures via Telegram bots.

Long story short, it was my aim to design my own perimetral alarm system, and no alarm system is a true alarm system without
its own surveillance cameras. Problem is, cameras cost and costs stack up rapidly...
unless you use the ESP32-CAM modules!

So there we go: ESP32 based camera system with the following features:

- 3 distinct webservers:
  - one server running on port 80 that lets the user remotely view and control the camera. 
    This was inherited from the default Arduino ESP32-Cam sketch.
  - one server running on port 81 is the MJPEG streaming page from inherited from the default Arduino ESP32-Cam sketch.
  - one very simple web server running on port 8080 with the purpose of controlling a few features of the general system such 
    as rebooting the device, turning of the camera, putting the system to sleep, taking a picture and sending it via Telegram, 
    turning the alarms on and off.
- ability to remotely configure certain aspects of the system such as storing the Telegram bot credentials and doing whatever 
  the server on port 8080 does via a very simple humand readable (the payload is a basic CSV string) UDP protocol
- the device broadcasts its own access point, but *also* connects to whatever upstream access points you specify, so that it 
  can be remotely controlled from anywhere else. The remote access points are hardcoded for security reasons and in case of 
  failover in a round-robin fashion.
- once the alarms are activated, either via web or UDP, the movement detection kicks in and case of movement an alarm message
  is sent via UDP to a configurable server. W.I.P.: also send a notice and the captured frame via telegram. 
- frames can be captured and sent via telegram on request via web and UDP.
- two remote management servers can be configured, these are part of a greater alarm system. The communication protocol
  is documented and designed to traverse NAT barriers. 
- the alarm mode with movement detection can be configured to kick in automatically in case contact with the management
  is lost but connectivity to the Internet is still available. 
