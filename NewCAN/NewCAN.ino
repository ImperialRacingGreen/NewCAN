// Required libraries
#include "variant.h"
#include <due_can.h>

#define CAN_BAUD_RATE CAN_BPS_250K
#define NDRIVE_RXID   0x210
#define NDRIVE_TXID	  0x180

#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x11BFFA4E

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

uint32_t sentFrames, receivedFrames;

//Leave this defined if you use the native port or comment it out if you use the programming port
//#define Serial SerialUSB

CAN_FRAME frame1, frame2, incoming;

CAN_FRAME frame_n_actual, frame_torque_cmd;

void printFrame(CAN_FRAME &frame) {
	Serial.print("ID: 0x");
	Serial.print(frame.id, HEX);
	Serial.print(" Len: ");
	Serial.print(frame.length);
	Serial.print(" Data: 0x");
	for (int count = 0; count < frame.length; count++) {
		Serial.print(frame.data.bytes[count], HEX);
		Serial.print(" ");
	}
	Serial.print("\r\n");
}

void populate_frames() {
	frame_n_actual.id = NDRIVE_RXID;
	frame_n_actual.length = 3;
	//frame_n_actual.data.low = 0x0064303d;
	//frame_n_actual.data.high = 0;
	frame_n_actual.data.bytes[0] = 0x3d;
	frame_n_actual.data.bytes[1] = 0x30;
	frame_n_actual.data.bytes[2] = 0x64;
	frame_n_actual.extended = 0;

	frame_torque_cmd.id = NDRIVE_RXID;
	frame_torque_cmd.length = 3;
	frame_torque_cmd.data.low = 0x0064903d;
	frame_torque_cmd.data.high = 0;
	frame_torque_cmd.extended = 0;
}

void abort_all_requests() {
	CAN_FRAME frame_abort, incoming;
	uint32_t counter = 0;
	
	frame_abort.id = NDRIVE_RXID;
	frame_abort.length = 3;
	frame_abort.data.bytes[0] = 0x3d;
	frame_abort.data.bytes[1] = 0x00; // REGID
	frame_abort.data.bytes[2] = 0xff;
	
	frame_abort.data.bytes[1] = 0x30;
	CAN.sendFrame(frame_abort);
	delayMicroseconds(100);

	frame_abort.data.bytes[1] = 0x90;
	CAN.sendFrame(frame_abort);
	delayMicroseconds(100);

	/*
	while (counter < 5000) {
		if (CAN.rx_avail()) {
			CAN.get_rx_buff(incoming);
			
			if (incoming.id == NDRIVE_TXID) {
				frame_abort.data.bytes[1] = incoming.data.bytes[0];
				CAN.sendFrame(frame_abort);
				delayMicroseconds(100);
			}
			
			counter = 0;
		} else {
			counter++;
		}
	}
	*/
}

/*
bool has_received_data(uint8_t data_address) {
	if (CAN.rx_avail()) {
		CAN.get_rx_buff(incoming);

		if (incoming.id == NDRIVE_TXID && incoming.data[0] == data_address) {
			return true;
		}

		delayMicroseconds(100);
	}
	
	return false;
}
*/

uint16_t pedal_reading() {
	return map(analogRead(A0), 40, 280, 0, 16380);
}

void setup() {

	// start serial port at 115200 bps:
	Serial.begin(115200);

	// Verify CAN0 and CAN1 initialization, baudrate set by CAN_BAUD_RATE:
	if (CAN.init(CAN_BAUD_RATE) &&
		CAN2.init(CAN_BAUD_RATE)) {
	}
	else {
	Serial.println("CAN initialization (sync) ERROR");
	}

	//Both of these lines create a filter on the corresponding CAN device that allows
	//just the one ID we're interested in to get through.
	//The syntax is (mailbox #, ID, mask, extended)
	//You can also leave off the mailbox number: (ID, mask, extended)
	CAN.setRXFilter(0, NDRIVE_TXID, 0x1FFFFFFF, false);
	CAN2.setRXFilter(0, NDRIVE_TXID, 0x1FFFFFFF, false);

	populate_frames();

	abort_all_requests();
}

// Test rapid fire ping/pong of extended frames
static void test_1(void)
{
	CAN_FRAME inFrame;
	uint32_t counter = 0;
        
	// Send out the first frame
	CAN.sendFrame(frame_n_actual);
	sentFrames++;

	while (1==1) {
		if (CAN.rx_avail()) {
			CAN.get_rx_buff(incoming);
			CAN.sendFrame(frame_n_actual);
			delayMicroseconds(100);
			sentFrames++;
			receivedFrames++;
			counter++;
		}
		if (CAN2.rx_avail()) {
			CAN2.get_rx_buff(incoming);
			CAN2.sendFrame(frame_n_actual);
			delayMicroseconds(100);
			sentFrames++;
			receivedFrames++;
			counter++;
		}
		if (counter > 5000) {
			counter = 0;
			Serial.print("S: ");
			Serial.print(sentFrames);
			Serial.print(" R: ");
			Serial.println(receivedFrames);
		}
	}
}

// can_example application entry point
void loop()
{
	CAN_FRAME incoming, test_frame, test_frame_2, test_frame_3;

	test_frame.id = NDRIVE_RXID;
	test_frame.length = 3;
	test_frame.data.bytes[0] = 0x3d;
	test_frame.data.bytes[1] = 0x30;
	test_frame.data.bytes[2] = 0x64;
	test_frame.extended = 0;

	test_frame_2.id = NDRIVE_RXID;
	test_frame_2.length = 3;
	test_frame_2.data.bytes[0] = 0x3d;
	test_frame_2.data.bytes[1] = 0x90;
	test_frame_2.data.bytes[2] = 0x64;
	test_frame_2.data.high = 0;
	test_frame_2.extended = 0;

	test_frame_3.id = NDRIVE_RXID;
	test_frame_3.length = 3;
	test_frame_3.data.bytes[0] = 0x90;
	test_frame_3.data.bytes[1] = 0xFC;
	test_frame_3.data.bytes[2] = 0x3F;

	//CAN.sendFrame(test_frame);
	//delayMicroseconds(100);
	//CAN.sendFrame(test_frame_2);
	//delayMicroseconds(100);
	uint8_t counter = 0;

	while (1) {
		
		uint16_t reading = pedal_reading();
		test_frame_3.data.bytes[1] = reading & 0xff;
		test_frame_3.data.bytes[2] = (reading >> 8) & 0xff;
		Serial.print(test_frame_3.data.bytes[1], HEX);
		Serial.print(" ");
		Serial.print(test_frame_3.data.bytes[2], HEX);
		Serial.print(" ");
		Serial.println(reading, HEX);

		printFrame(test_frame_3);
		delayMicroseconds(100);
		CAN.sendFrame(test_frame_3);
		delayMicroseconds(100);
		delay(1000);

		if (CAN.rx_avail()) {
			CAN.get_rx_buff(incoming); 
			printFrame(incoming);
		}
		if (CAN2.rx_avail()) {
			CAN2.get_rx_buff(incoming); 
			printFrame(incoming);
		}
	}
}
