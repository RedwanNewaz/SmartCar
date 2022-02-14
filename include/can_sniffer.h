/*
CanSniffer is developed based on https://www.youtube.com/watch?v=ZhYc95b6WoU

Mazda 3's CAN protocols
    http://she-devel.com/Mazda3_Controller_Area_Network_Experimentation.html
    HS CAN (ISO 11898, 11-bit Tx, 500kbps, var DLC) on pins 6, 14 and 5 (?)
    I type: "STP31; STPRS"
    Vehicle responds: "HS CAN (ISO 11898, 500K/11B)"
    MS CAN (ISO 11898, 11-bit Tx, 125kbps, var DLC) on pins 3, 11 and 5 (?)
    I type: "STP51;STPRS":
    Vehicle responds: "MS CAN (ISO 11898, 125K/11B)"
Mazda 3's OBD2 pinout 
            MS               HS
            HI   GND   COM   HI
    1   2    3    4     5    6    7   8

    9   10   11   12   13   14   15   16
             MS             HS        +12V  
             LO             LO
*/

#include <Arduino.h>
#include <esp32_can.h>
#include "state_machine.h"
#include "mqtt_client.h"

#define RANDOM_CAN 1

extern MqttClient *mqtt; 

class CanSnifferBase{

public:
        // Inits, globals
    typedef struct {
        long id;
        byte rtr;
        byte ide;
        byte dlc;
        byte dataArray[20];
    } packet_t;
    
    //------------------------------------------------------------------------------
    // CAN packet simulator
    void CANsimulate(void) {
        packet_t txPacket;

        long sampleIdList[] = {0x110, 0x18DAF111, 0x23A, 0x257, 0x412F1A1, 0x601, 0x18EA0C11};
        int idIndex = random (sizeof(sampleIdList) / sizeof(sampleIdList[0]));
        int sampleData[] = {0xA, 0x1B, 0x2C, 0x3D, 0x4E, 0x5F, 0xA0, 0xB1};

        txPacket.id = sampleIdList[idIndex];
        txPacket.ide = txPacket.id > 0x7FF ? 1 : 0;
        txPacket.rtr = 0; //random(2);
        txPacket.dlc = random(1, 9);

        for (int i = 0; i < txPacket.dlc ; i++) {
            int changeByte = random(4);
            if (changeByte == 0) {
            sampleData[i] = random(256);
            }
            txPacket.dataArray[i] = sampleData[i];
        }

        printPacket(&txPacket);
    }
    //------------------------------------------------------------------------------
    // GUI interface 
    void RXcallback(void) {
        int rxPtr = 0;
        char rxBuf[RXBUF_LEN];
        // this function invoke from the pc gui 
        while (Serial.available() > 0) {
            if (rxPtr >= RXBUF_LEN) {
                rxPtr = 0;
            }
            char c = Serial.read();
            rxBuf[rxPtr++] = c;
            if (c == TERMINATOR) {
                rxParse(rxBuf, rxPtr);
                rxPtr = 0;
            }
        }
    }

    virtual void sendPacketToCan(packet_t * packet) = 0;

protected:

    const char SEPARATOR = ',';
    const char TERMINATOR = '\n';
    const char RXBUF_LEN = 100;
    const int QUEUE_SZ = 10;
    const  int PACKET_LEN =  8;  


    //------------------------------------------------------------------------------
    // // Printing a packet to serial
    // void printHex(long num) {
    //     if ( num < 0x10 ){ Serial.print("0"); }
    //     Serial.print(num, HEX);
    // }

    // void printPacket(packet_t * packet) {
    //     // packet format (hex string): [ID],[RTR],[IDE],[DATABYTES 0..8B]\n
    //     // example: 014A,00,00,1A002B003C004D\n
    //     printHex(packet->id);
    //     Serial.print(SEPARATOR);
    //     printHex(packet->rtr);
    //     Serial.print(SEPARATOR);
    //     printHex(packet->ide);
    //     Serial.print(SEPARATOR);
    //     // DLC is determinded by number of data bytes, format: [00]
    //     for (int i = 0; i < packet->dlc; i++) {
    //         printHex(packet->dataArray[i]);
    //     }
    //     Serial.print(TERMINATOR);
    // }

    void printPacket(packet_t * packet) {
        // packet format (hex string): [ID],[RTR],[IDE],[DATABYTES 0..8B]\n
        // example: 014A,00,00,1A002B003C004D\n
        auto printHex = [](long num){
            String result = ( num < 0x10 )?"0":"";
            result +=  String(num, HEX);
            return result;
        };
        String msg = printHex(packet->id);
        msg += SEPARATOR; 
        msg += printHex(packet->rtr);
        msg += SEPARATOR; 
        msg += printHex(packet->ide);
        msg += SEPARATOR; 
        // DLC is determinded by number of data bytes, format: [00]
        for (int i = 0; i < packet->dlc; i++) {
            msg += printHex(packet->dataArray[i]);
        }
        msg += TERMINATOR;
        
        // bin_sem = xSemaphoreCreateBinary();
        // xSemaphoreTake(bin_sem, portMAX_DELAY);
        mqtt->publishSerialData(msg.c_str());
    }

    //------------------------------------------------------------------------------
    // Serial parser
    char getNum(char c) {
        if (c >= '0' && c <= '9') { return c - '0'; }
        if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
        if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
        return 0;
    }

    char * strToHex(char * str, byte * hexArray, byte * len) {
        byte *ptr = hexArray;
        char * idx;
        for (idx = str ; *idx != SEPARATOR && *idx != TERMINATOR; ++idx, ++ptr ) {
            *ptr = (getNum( *idx++ ) << 4) + getNum( *idx );
        }
        *len = ptr - hexArray;
        return idx;
    }

    void rxParse(char * buf, int len) {
        packet_t rxPacket;
        char * ptr = buf;
        // All elements have to have leading zero!

        // ID
        byte idTempArray[8], tempLen;
        ptr = strToHex(ptr, idTempArray, &tempLen);
        rxPacket.id = 0;
        for (int i = 0; i < tempLen; i++) {
            rxPacket.id |= idTempArray[i] << ((tempLen - i - 1) * 8);
        }

        // RTR
        ptr = strToHex(ptr + 1, &rxPacket.rtr, &tempLen);

        // IDE
        ptr = strToHex(ptr + 1, &rxPacket.ide, &tempLen);

        // DATA
        ptr = strToHex(ptr + 1, rxPacket.dataArray, &rxPacket.dlc);

        #if RANDOM_CAN == 1
        // echo back
        printPacket(&rxPacket);
        #else
        sendPacketToCan(&rxPacket);
        #endif
    }

};


class CanSniffer: public CanSnifferBase{
public:
    //------------------------------------------------------------------------------
    // CAN RX, TX
    /**
     * @brief https://github.com/sandeepmistry/arduino-CAN/blob/master/API.md
     * id - 11-bit id (standard packet) or 29-bit packet id (extended packet)
     * dlc - (optional) value of Data Length Code (DLC) field of packet, default is size of data written in packet
     * rtr - (optional) value of Remote Transmission Request (RTR) field of packet (false or true), defaults to false. 
     * RTR packets contain no data, the DLC field of the packet represents the requested length.
     * @param rx_frame 
     */
    CanSniffer()
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // Serial.println("Doing Auto Baud scan on CAN0");
        initialize_ = CAN0.beginAutoSpeed() == 0 ? false: true;
      
        //By default there are 7 mailboxes for each device that are RX boxes
        //This sets each mailbox to have an open filter that will accept extended
        //or standard frames
        int filter;
        //extended
        for (filter = 0; filter < 3; filter++) {
            CAN0.setRXFilter(filter, 0, 0, true);
            
        }  
        //standard
        for (int filter = 3; filter < 7; filter++) {
            CAN0.setRXFilter(filter, 0, 0, false);
           
        }  
    }
    //------------------------------------------------------------------------------
    // CAN RX, TX
    /**
     * @brief https://github.com/sandeepmistry/arduino-CAN/blob/master/API.md
     * id - 11-bit id (standard packet) or 29-bit packet id (extended packet)
     * dlc - (optional) value of Data Length Code (DLC) field of packet, default is size of data written in packet
     * rtr - (optional) value of Remote Transmission Request (RTR) field of packet (false or true), defaults to false. 
     * RTR packets contain no data, the DLC field of the packet represents the requested length.
     * @param rx_frame 
     */
    void onCANReceive(const CAN_FRAME &rx_frame) {
        // received a CAN packet

        packet_t rxPacket;
        rxPacket.id = rx_frame.id;
        rxPacket.rtr = rx_frame.rtr; // Remote Transmission Request (RTR) field of packet (false or true), defaults to false. 
        rxPacket.ide = rx_frame.extended; // check if this is a standard or extended CAN frame
        rxPacket.dlc = rx_frame.length;
        
     
       
        
        for(int i = 0; i < rx_frame.length; ++i) {
            rxPacket.dataArray[i] = (byte) rx_frame.data.bytes[i];         
        }
        printPacket(&rxPacket);
    }

    void sendPacketToCan(packet_t * packet) {
        // transmit a CAN packet

        CAN_FRAME tx_frame;
        tx_frame.id = packet->id; 
        tx_frame.extended = packet->ide;
        tx_frame.length = packet->dlc;
        tx_frame.rtr = packet->rtr;


        for(int i = 0; i < PACKET_LEN; ++i) {
            tx_frame.data.bytes[i] = (uint8_t) packet->dataArray[i];         
        }
        // CAN0.write(tx_frame);
    }

    /**
     * @brief based on https://github.com/adamtheone/canDrive/blob/main/01_canSniffer_Arduino/canSniffer/canSniffer.ino
     * make sure disable serial print from vehicle_control.h or other module 
     * read can msg from car https://github.com/nhatuan84/esp32-can-protocol-demo/blob/master/examples/esp32cansend/esp32cansend.ino
     */
    void run()
    {
        
        while(initialize_)
        {   
            CAN_FRAME incoming;
            bool canMsgFound = false; 
            if (CAN0.available() > 0) {
                canMsgFound = true; 
                CAN0.read(incoming); 
                onCANReceive(incoming);
            }
            

            // simulation 
            #if RANDOM_CAN == 1
                CANsimulate();
                delay(100);
            #endif
        } 
    }

    void update()
    {

        // simulation 
        #if RANDOM_CAN == 1
            CANsimulate();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        #else 
            if(initialize_)
            {   
                CAN_FRAME incoming;
                bool canMsgFound = false; 
                if (CAN0.available() > 0) {
                    canMsgFound = true; 
                    CAN0.read(incoming); 
                    onCANReceive(incoming);
                }
            }
            else
            {
                printf("CAN0 did not initialize reboot \n");
                vTaskDelay(1000);
            }
        #endif


    }

    /**
     * @brief update shared msg status 
     * 
     * @param packet 
     */
    void update_state_machine(packet_t * packet)
    {
        // don't update any thing when vechicle control is running 
        if(shared_msg->cntrl == controller_state::RUNNING) return; 
    }
private:
    bool initialize_;

};


//Task1code: read can bus message from obd2 port 
inline void onCanBusDecoder( void * pvParameters ){
  CanSniffer mazdaCAN;
  // it will run continuously 
  mazdaCAN.run();

}
