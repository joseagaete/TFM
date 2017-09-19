/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device and prints data
 *
 *
 */

/*
 *  LoRa 868 / 915MHz SX1272 LoRa module
 *
 *  Instituto de Desarrollo e Innovacio•n en Comunicaciones (IDeTIC)
 *  Departamento de Se√±ales y Comunicaciones
 *  Escuela de Ingenieria de Telecomunicacion y Electronica
 *  Master Universitario en Ingenieria de Telecomunicacion
 *  Univesidad de las Palmas de Gran Canaria (ULPGC)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Version:           1.0
 *  Design:            Jose Antonio Godoy Rosario
 *  Implementation:    Jose Antonio Godoy Rosario
 *  Lenguaje:	       C++
 *  Descripcio•n 	Prograna que se ejecuta en el Transmisor LoRa. Dicho programa evia
 *  un mensaje Unicast a la direccio•n especificada y seguidamente un mensaje en Broadcat
 *  ambos con la posicion del UAV (Unmaned Aerial Vehicle)
 */




#include "./mavlink/include/mavlink/v1.0/common/mavlink.h"
//#include "/home/pi/AttitudePositionSerialMavlink/mavlink/include/mavlink/v1.0/common/mavlink.h"

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

//Include the SX1272 and SPI librarie
#include "arduPiLoRa.h"
#include "string"
//#include <inttypes>

//#define  COUNTER  50
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

// Settings
int sysid = 42; ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
int fd;

//LoRa variables
float lat, lon, alt;
int e;
char message1 [100];
char message2 [] = "Packet2, broadcast test";
int8_t mode = 1;
char power;
char enter;
int destino = 0;
int direccion = 2;

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(const char* port) {
        fflush(stdout);

	int fd; /* File descriptor for the port */

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		/* Could not open the port. */
		return (-1);
	} else {
		fcntl(fd, F_SETFL, 0);
	}
	//cout << "Puerto abierto con fd: " << fd << endl;
	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity,
		bool hardware_control) {
	//struct termios options;

	struct termios config;
	if (!isatty(fd)) {
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n",
				fd);
		return false;
	}
	if (tcgetattr(fd, &config) < 0) {
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	ONOCR | OFILL | OPOST);

#ifdef OLCUC
	config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
	config.c_oflag &= ~ONOEOT;
#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN] = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud) {
	case 1200:
		if (cfsetispeed(&config, B1200) < 0
				|| cfsetospeed(&config, B1200) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 1800:
		cfsetispeed(&config, B1800);
		cfsetospeed(&config, B1800);
		break;
	case 9600:
		cfsetispeed(&config, B9600);
		cfsetospeed(&config, B9600);
		break;
	case 19200:
		cfsetispeed(&config, B19200);
		cfsetospeed(&config, B19200);
		break;
	case 38400:
		if (cfsetispeed(&config, B38400) < 0
				|| cfsetospeed(&config, B38400) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 57600:
		if (cfsetispeed(&config, B57600) < 0
				|| cfsetospeed(&config, B57600) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 115200:
		if (cfsetispeed(&config, B115200) < 0
				|| cfsetospeed(&config, B115200) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
	case 460800:
		if (cfsetispeed(&config, 460800) < 0
				|| cfsetospeed(&config, 460800) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	case 921600:
		if (cfsetispeed(&config, 921600) < 0
				|| cfsetospeed(&config, 921600) < 0) {
			fprintf(stderr,
					"\nERROR: Could not set desired baud rate of %d Baud\n",
					baud);
			return false;
		}
		break;
	default:
		fprintf(stderr,
				"ERROR: Desired baud rate %d could not be set, aborting.\n",
				baud);
		return false;

		break;
	}

	//
	// Finally, apply the configuration
	//
	if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}




void close_port(int fd) {
	close(fd);
}



/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
mavlink_global_position_int_t  serial_wait() {
//	int fd = serial_fd;

mavlink_status_t lastStatus;
lastStatus.packet_rx_drop_count = 0;

int timer = 0;
int  COUNTER = 1000;
float global_position_array[4];   
float satellites;
//char *uart_name = (char*) "/dev/ttyAMA0";
char *uart_name = (char*) "/dev/ttyACM0";

//int baudrate = 57600;
int baudrate = 115200;
fd = open_port(uart_name);
bool setup = setup_port(fd, baudrate, 8, 1, false, false);

	// Blocking wait for new data
//	while (1) {
       while(timer<COUNTER){
		//if (debug) printf("Checking for new data on serial port\n");
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;
		if (read(fd, &cp, 1) > 0) {
                        // Check if a message could be decoded, return the message in case yes
                        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message,
                                        &status);
                        if (lastStatus.packet_rx_drop_count
                                        != status.packet_rx_drop_count) {
                                if (verbose || debug)
                                        printf("ERROR: DROPPED %d PACKETS\n",
                                                        status.packet_rx_drop_count);
                                if (debug) {
                                        unsigned char v = cp;
                                        fprintf(stderr, "%02x ", v);
                                }
                        }
                        lastStatus = status;
                } else {
                        if (!silent)
                                fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
                }



		// If a message could be decoded, handle it
		if (msgReceived) {
			//if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;



			/* decode and print */

			// For full MAVLink message documentation, look at:
			// https://pixhawk.ethz.ch/mavlink/
			// Only print every n-th message
			switch (message.msgid) {
			 case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{
                                // decode
                                mavlink_global_position_int_t position;
                                mavlink_msg_global_position_int_decode(&message, &position);
				
				close_port(fd);
					
				return position;
                                 

				
				
								

                                }
                                break;
				

			}

		}
	
          timer++;
	}
     
	
    }


/* print the global_position information*/

float getLat()
{

         mavlink_global_position_int_t global_pos;
         global_pos=serial_wait();

         return (float)global_pos.lat/10000000;

}

float getLon()
{
        mavlink_global_position_int_t global_pos;
        global_pos=serial_wait();
        return (float)global_pos.lon/10000000;

}

float getAlt()
{
      mavlink_global_position_int_t global_pos;
      global_pos=serial_wait();
      return (float)global_pos.alt/1000;

}



void setup(){
    //Print a start message
    printf("SX1272 module and Raspberry PI: send packets with ACK \n");
    
    //Power ON the module
    e = sx1272.ON();
    printf("Setting power ON: state %d \n",e);
    
    //Set transmission mode
    printf("Introduzca el modo deseado: \n");
    scanf("%i", &mode);
    printf("Ha seleccionado el modo: %i \n",mode);
    e |= sx1272.setMode(mode);
    printf("Setting Mode: state %d \n",e);
    
    
    // Set header
    e |= sx1272.setHeaderON();
    printf("Setting Header ON: state %d\n", e);
    
    // Select frequency channel
    e |= sx1272.setChannel(CH_14_868);
    printf("Setting Channel: state %d\n", e);
    printf("\n");
    printf("\n");
    delay(2000);
    
    // Set CRC
    e |= sx1272.setCRC_ON();
    printf("Setting CRC ON: state %d\n", e);
    
    // Select output power (Max, High or Low)
    
    seleccionPotencia:
        printf ("Seleccione el nivel de potencia (L,H,M): \n");
        scanf("%c",&enter);
        scanf ("%c", &power);
    
        if(power ==  'L' || power ==  'M' || power == 'H'){
        
            printf ("Ha seleccionado potencia: %c \n", power);
            e |= sx1272.setPower(power);
            printf("Setting Power: state %d\n", e);
        }else{
            printf ("el valor introducido no es valido \n");
            goto seleccionPotencia;
   	 }
    
    // Set the node address
    scanf ("%c",&enter);
        SeleccionDireccion:
        printf("Seleccione la direcci‚àö‚â•n desead \n");
        scanf ("%i", &direccion);
    
        if (direccion == 1 || direccion == 0 || direccion >255){
            printf("Direccion no disponible o erronea \n");
            goto SeleccionDireccion;
        
        }else{
            e |= sx1272.setNodeAddress(direccion);
            printf("Setting Node address: state %d\n", e);
        }
    
    //set dest,
    scanf("%c", &enter);
    SeleccionDestino:
        printf("Seleccione el destino deseado: \n");
        scanf("%i", &destino);
        if (destino == direccion){
            printf ("RX y TX son el mismo. ERROR \n");
            goto SeleccionDestino;
    
        }else{
            printf ("destino seleccionado: %d \n", &destino);
        }
    
    // Print a success message
    if (e == 0)
        printf("SX1272 successfully configured\n");
    else
          printf("SX1272 initialization failed\n");
    
    delay(1000);
    
    // Show config
    printf("************************************************\n");
    printf("Configuracion del Transmisor correcta \n");
    printf("***********************************************\n");

}
void loop(void)
{
    // Send message1 and print the result
    
    sprintf(message1,"Long: %f, lat: %f, alt: %f",lon,lat,alt); 
    printf("enviando mensaje al nodo... %d \n", destino);
    e = sx1272.sendPacketTimeoutACK(destino, message1);
    printf("Packet sent, state %d\n",e);
    
    if(e == 0){
        printf("ACK recibido, comunicacion correcta \n");
    }else{
        printf("ACK no recibido, error en la comunicacion \n");
    }
    delay(4000);
    
    // Send message2 broadcast and print the result
    printf ("Enviando mensaje en broadcast... \n");
    e = sx1272.sendPacketTimeoutACK(0, message2);
    printf("Packet sent, state %d\n",e);    
    if (e == 0){
        printf("ACK recibido, comunicacion correcta \n");
    }else{
        printf("ACK no recibido, Error en la comunicacion \n");
    }
    
    delay(4000);
}




int main()
 {
     setup();
     while(1){
        
         cout << "Empezamos!!!" << endl;
         
         lat=getLat(); //obtengo lat
         lon=getLon(); //obtengo long
         alt=getAlt(); //obtengo alt
         
         
         cout <<   "Usando las nuevas funcines" << endl;
         cout << "Latitud: " << lat << "Longitud: " << lon << "Altitud: " << alt << endl;
         
         loop(); //mando el mensaje por LoRa
         
     }

  return 0;
}


