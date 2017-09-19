/*  
 *  LoRa 868 / 915MHz SX1272 LoRa module
 *  
 *  Instituto de Desarrollo e Innovación en Comunicaciones (IDeTIC)
 *  Departamento de Señales y Comunicaciones
 *  Escuela de Ingeniería de Telecomunicación y Electrónica 
 *  Máster Universitario en Ingeniería de Telecomunicación
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
 *  Design:            nJos�Antonio Godoy Rosario 
 *  Implementation:    CoJos�é Antonio Godoy Ros
 *  Lenguaje:	       C++
 *  Descripción 	Prograna que se ejecuta en el Transmisor LoRa. Dicho programa enbv��ía
 *  un mensaje Unicast a la dirección especificada y seguidamente un mensaje en Broadcast
 */
 


/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////IMPORTANTE: Falta el código de integración con controlador Ardupilot para recibir mensajes MAINK//////////
  ///////con los datos de POS/Y/P/R..									   //////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 */


// Include the SX1272 and SPI library: 
#include "arduPiLoRa.h"
#include <iostream>
#include "string"
#include <inttypes.h>

int e;

char message1 [] = "Packet 1, wanting to see if received packet is the same as sent packet";
char message2 [] = "Packet 2, broadcast test";
char message [100];
float lon = 8.172;
float lat =  2.423;
float alt = 500;
int8_t mode = 1;
char power;
char enter;
int destino = 0;
int direccion = 2;
void setup()
{
  // Print a start message
	  printf("SX1272 module and Raspberry Pi: send packets with ACK\n");
  
  // Power ON the module
 ;	 e = sx1272.ON();
 	 printf("Setting power ON: state %d\n", e);
  
  // Set transmission mode
	 printf ("introduzca el modo desado: \n");
	 scanf ("%i", &mode);
	 printf ("ha seleccionado el modo: %i \n",mode);
        e |= sx1272.setMode(mode);
 	printf("Setting Mode: state %d\n", e);
  
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
	printf("Seleccione la dirección desead \n");
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
	printf("Configuraci�n del Transmisor correcta \n");
	printf("***********************************************\n");
}

void loop(void)
{
	// Send message1 and print the result
    printf("enviando mensaje al nodo... %d \n", destino);
    sprintf(message,"long: %f, lat: %f, alt: %f" ,lon,lat,alt);
    printf (message);
    printf ("\n");
    e = sx1272.sendPacketTimeoutACK(destino, message);
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

int main (){
	setup();
	while(1){ //bucle infinito
		loop();
	}
	return (0);
}
