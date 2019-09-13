These are some config files required to setup the raspberry pi.
When raspberry pi first flashed; ssh, NetworkManager, and wpa_supplicant must me place on the boot partion befor first boot


wpa_supplicant.conf
    file to store wifi credentials

mac.txt 
    Keeps track of last ip address and mac address (static)
    
ssh.txt
    login info for logging into raspberry pi remotely
    
ssh
    blank file for initializing ssh

NetworkManager.conf
    creates a static mac
    
