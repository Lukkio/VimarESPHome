This is the stock firmware patched with those features:

Read over UART:
- Actual temperature exposed via UART;
- Setted temperature exposed via UART;
- On/off status exposed via UART;
- Heat/Cold mode exposed via UART;
- °C/°F mode exposed via UART;

Write over UART:
- Setpoint temperature;
- On/off status;
- Heat/Cold mode;
- °C/°F mode;


**PayloadBlob project is only the BLOB of the firmware feature. Manual firmware patch is required.

//firmware patch offset 0x80064b4 -> 0x080084f8
//Payload offset -> 0x080084f8 (GetParamert())

Ram:
//	temperatura settata -> 0x20000680 (valore int)
//	temperatura attuale -> 0x20000650 (valore int)
//	on/OFF -> 0x2000067C (valore 0x00 / 0x01)
//	luminosità -> 0x20000670 (valore 0x00 / 0x01 / 0x02 / 0x03)
//	caldo/freddo -> 0x2000066C (valore 0x00 / 0x01)
//	celsus/Fahre.. -> 0x20000671 (valore 0x00 / 0x01)
//	display visualizzato -> 0x20000011 (3 char)

//Commands UART read:
//"ReadA"  -> read all variables
//Buffer	   [0][1]	    |	[2]   |	 [3]  |		[4]		 |		[5][6]		 | [7] |
//		| ActualTemperature | HotCold | OnOff | Celsus_Fahre | SettedTemperature | \n  |

//Commands UART write:
//"Set 1"
//"Set 2"	HotCold -> command UART to send: "Set2 [0x01,0x00]"
//"Set 3"	Brightness -> command UART to send: "Set3 [0x01,0x02,0x03]"
//"Set 4"	OnOff -> command UART to send: "Set4 [0x01,0x00]"
//"Set 5"	Celsus_Fahre -> command UART to send: "Set5 [0x01,0x00]"  
//"Set 6"	SettedTemperature2 -> command UART to send: "Set6 [0xff,0xff]"
//"Set 7"	OnOff -> command UART to send: "Set7 [OnOff(0x01),HotCold(0x00)]"
//"Set 8 BA" button software set test
//"Set 9"	OnOff -> command UART to send: "Set9 [0x01,0x00]" + 12c memory write
//"BOOTL"	jump to bootloader