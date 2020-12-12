# Description

It is a time-consuming job to design various vibrotactile patterns for games.

Haptic designers often choose to design monotonous patterns like **Haptic.vibrate(50ms, amp=0.3)**.

We want to make a tool that makes haptic designers design well vibrotactile patterns without cumbersome procedures.

PressureGlove is trying to capture the force applied on hands and re-mapping to vibration motors.

Haptic designers just need to adjust the force curves captured by PressureGlove slightly.

For more details, please refer to our paper [IEEE AIVR 2019](https://ieeexplore.ieee.org/document/8942381/authors) .

# Supported Board

ESP8266-Based Board (Including NodeMCU)

Linkit 7697 (Not recommended due to its buffer-overflow issues)

# Hardware Details

## Pressure Sensors

We use FSR402, a cheap round force-sensing resistor, to measure the pressure on the fingertips.

## Analog-Digital Converter

ADS1115 is a 16-bit ADC, which means it can provide precise voltage data in our application.

## MUX

A 16-channel MUX, CD74HC4067, is used to extend the number of ports from the ADC module.


# Software Details

## Wi-Fi Connection

The SSID and Wi-Fi password are modifiable in compile time.

## mDNS

It did support mDNS before.

However, due to the lack of mDNS supports on Windows, the feature was deprecated. 

Instead, it supports UDP broadcast for PCs to get the device's IP.

## UDP Broadcast

After it connects to Wi-Fi, it will create a UDP broadcast listener.

PCs need to send a packet that contains its TCP port (The UDP Packet Headers contain Source IPs).

After parsing the UDP broadcast packet, it will create a socket connection to the PC. 

# Sources of Arduino Libraries

[ADS1115](https://github.com/baruch/ADS1115)

# PC Side Code

Broadcast Packet


        private static int _ServerPort = 12700;
		
        private static byte[] BroadcastPacket = //FF EE SP_L SP_H 
		
        {
		
            (byte)0xFF,
			
            (byte)0xEE,
			
            (byte)(_ServerPort & 0xFF),
			
            (byte)(_ServerPort >> 8)
			
        };
        

Send UDP Broadcast Packet in C#

        private static void BroadcastMessage(byte[] message, int port)
		
        {
		
            using (var sock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
			
            {
			
                sock.EnableBroadcast = true;
				
                sock.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, true);
				
                IPEndPoint iep = new IPEndPoint(IPAddress.Broadcast, port);
				
                sock.SendTo(message, iep);
				
            }
			
        }