#!/bin/bash

# Here I inquire super user permissions.

if [ $EUID != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi


# Script assumes that user does not have dns server before.
# Install light weight dns server for small networks.
# If I am not mistaken it will launch with error after installation.

sudo apt install dnsmasq

# Configure dns server. Later it should be replaced with regular expressions.
sudo echo "port=53" >> /etc/dnsmasq.conf
sudo echo "domain-needed" >> /etc/dnsmasq.conf
sudo echo "bogus-priv" >> /etc/dnsmasq.conf
sudo echo "strict-order" >> /etc/dnsmasq.conf
sudo echo "bind-interfaces" >> /etc/dnsmasq.conf
sudo echo "interface=eth0" >> /etc/dnsmasq.conf
sudo echo "dhcp-range=10.5.5.80,static" >> /etc/dnsmasq.conf
# Important line. It means give the host with specific name special static ip.
# Particulary lidar says its name, and we assign special ip to it.
# Name of ouster lidar is os1-991*. You may find specific digits on top of it. 
sudo echo "dhcp-host=os1-991936000848,10.5.5.80,infinite" >> /etc/dnsmasq.conf

# Now we must configure host file in such way that we will introduce special name
# to ip address what our dns provided to our lidar.
sudo echo "10.5.5.80	os1-991936000848" >> /etc/hosts

# Modification of standart network manager that described in ouster vendor documentation
# Here we assign constant ip to ethernet interface.
# It is required for proper work of dhcp server
sudo touch /etc/network/interfaces.d/eth0
sudo echo "auto lo eth0" >> /etc/network/interfaces.d/eth0
sudo echo "iface lo inet loopback" >> /etc/network/interfaces.d/eth0
sudo echo "iface eth0 inet static" >> /etc/network/interfaces.d/eth0
sudo echo "	address 10.5.5.1" >> /etc/network/interfaces.d/eth0
sudo echo "	netmask 255.255.255.0" >> /etc/network/interfaces.d/eth0

# restart dns server with new configuration.
sudo systemctl restart NetworkManager
sudo systemctl restart dnsmasq




