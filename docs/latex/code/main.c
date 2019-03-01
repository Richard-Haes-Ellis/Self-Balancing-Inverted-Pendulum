int main(void)
{
bool bLastSuspend;
uint32_t ui32SysClock;
uint32_t ui32PLLRate;

// Run from the PLL at 120 MHz.
ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),120000000);

// Configuramos el boosterpack
Conf_Boosterpack(BP, ui32SysClock);

// Configuramos los pines de la uart (ETHERNET|UART)
PinoutSet(false, true);

// Inicializamos los botones de la placa
ButtonsInit();

// Habilitamos el periferico UART0
ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

// Inicializamos la UART para la consola .
UARTStdioConfig(0, 115200, ui32SysClock);

// Inicialmente el raton estara desconfigurado
g_bConnected = false;
g_bSuspended = false;
bLastSuspend = false;

// Inicializamos el stack del USB para el modo dispositivo
USBStackModeSet(0, eUSBModeDevice,      0);

// Le decimos a la libreria USB el clock de la CPU y la frecuncia de la PLL
// Es requerido para las placas TM4C129.
SysCtlVCOGet(SYSCTL_XTAL_25MHZ,            &ui32PLLRate);
USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &ui32SysClock);
USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

// Pasamos informacion de nuestro dispositivo al dirver USB HID
// Inicializamos el controlador USB y conectamos el dispositvo al bus
USBDHIDMouseInit(0, (tUSBDHIDMouseDevice *)&g_sMouseDevice);

// Configuramos el reloj del sistema para contar 100 veces por segudo
ROM_SysTickPeriodSet(ui32SysClock / SYSTICKS_PER_SECOND);
ROM_SysTickIntEnable();
ROM_SysTickEnable();

// Mensaje Inicial
UARTprintf("\033[2J\033[H\n");
UARTprintf("******************************\n");
UARTprintf("*         usb-mouse	         *\n");
UARTprintf("******************************\n");

// Comprobamos el funcionamiento del sensor
UARTprintf("\033[2J \033[1;1H Inicializando BMI160... ");
cod_err = Test_I2C_dir(2, BMI160_I2C_ADDR2);
if (cod_err)
{
	// Fallo del sensor
	UARTprintf("Error 0X%x en BMI160\n", cod_err);
	Bmi_OK = 0;
}
else
{
	// Exito
	UARTprintf("Inicializando BMI160, modo NAVIGATION... ");
	bmi160_initialize_sensor();
	bmi160_config_running_mode(APPLICATION_NAVIGATION);
	UARTprintf("Hecho! \nLeyendo DevID... ");
	readI2C(BMI160_I2C_ADDR2, BMI160_USER_CHIP_ID_ADDR, &DevID, 1);
	UARTprintf("DevID= 0X%x \n", DevID);
	Bmi_OK = 1;
}

/* BUCLE PRINCIPAL ****************************************************/
/* Empezamos esperando a que se conecte el micro a algun host , luego */
/* Luego entramos en el manejador de botones y movimiento del sensor  */
/* Si por lo que sea nos desconectamos del host volvemos a la espera  */
/**********************************************************************/
while (1)
{
uint8_t ui8Buttons;
uint8_t ui8ButtonsChanged;

UARTprintf("\nEsperamos al host...\n");

// Indica que el micro aun no esta listo para usar
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);

// Nos quedamos esperado si no esta conectado al host (PC)
while (!g_bConnected){}

// Indica que el micro esta listo para usar
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 1);

// Una vez connectada informamos por UART
UARTprintf("\nHost conectado...\n");

// Marcamos el estado de espera
g_iMouseState = STATE_IDLE;

// Declaramos variable de botones
uint8_t currB1State,prevB1State = 0;
uint8_t currB2State,prevB2State = 0;
uint8_t butReport = 0;

// En principio marcamos como bus no suspenso (Ya que nos acabamos de conectar)
bLastSuspend = false;

// Continuamos con nuestra logica de programa (Funcionnalidad del raton)
// mientras estamos conectados. Esta variable es manejada por el MouseHandler
// en funcion de los eventos que ocurran (Conexion/Desconecion/Suspension..)
while (g_bConnected)
{
	// Comprobamos si el estado de suspenso ha cambiado
	if (bLastSuspend != g_bSuspended)
	{
		// En caso de que si informamos por UART
		bLastSuspend = g_bSuspended;
		if (bLastSuspend)
		{
			UARTprintf("\nBus Suspended ... \n");
		}
		else
		{
			UARTprintf("\nHost Connected ... \n");
		}
	}
	// Si estamos en el estado de espera podemos realizar las funcionalidades normales
	if (g_iMouseState == STATE_IDLE)
	{
		// Si ha pasado mas de 10 ms actualizamos
		if (g_ui32SysTickCount - g_ui32PrevSysTickCount > 1)
		{
			// Reseteamos el cotador
			g_ui32PrevSysTickCount = g_ui32SysTickCount;
			// Si el sensor esta bien
			if (Bmi_OK)
			{
				// Leemos los datos por I2C
				bmi160_read_gyro_xyz(&s_gyroXYZ);

				// Filtramos los datos
				xdata = filter(s_gyroXYZ.x-gyro_off_x,xfilterBuff);
				ydata = filter(s_gyroXYZ.z-gyro_off_z,yfilterBuff);

				// QUE RANGO TOMA s_gyroXYX ? ----> 16 bits!!!
				// Se DEBE escalar desde -32768 a 32767. Lo hacemos por casting
				if((xdata/scaling) < thresh && (xdata/scaling) > -thresh)
				{	// Si esta dentro del umbral [-thresh,thres] rechazamos
					yDistance = 0;
				}
				else
				{	// En cualquier otro caso lo aceptamosy escalamos el dato
					yDistance = -(int8_t)(xdata/scaling);
				}

				// Lo mismo para el eje x
				if((ydata/scaling) < thresh && (ydata/scaling) > -thresh)
				{
					xDistance = 0;
				}
				else
				{
					xDistance = -(int8_t)(ydata/scaling);
				}

				// Indicamos entonces que el raton se ha movido
				movChange = 1;
			}
		}

		// Comprobamos si los botones han sido pulsados
		ButtonsPoll(&ui8ButtonsChanged, &ui8Buttons);

		// Actualizamos las variables de estado de los botones
		currB1State = (ui8Buttons & LEFT_BUTTON);
		currB2State = (ui8Buttons & RIGHT_BUTTON);

		butChange = 0;

		// Detectamos flancos de subida o bajada
		if (currB1State && !prevB1State) 	// SUBIDA (0->1)
		{
			prevB1State = 1; // Actualizamos el valor posterior
			butChange   = 1; // Indicamos cambio de estado
			butReport = MOUSE_REPORT_BUTTON_2;
		}
		else if (!currB1State && prevB1State) // BAJADA (1->0)
		{
			prevB1State = 0; // Actualizamos el valor posterior
			butChange   = 1; // Indicamos cambio de estado
			butReport = MOUSE_REPORT_BUTTON_RELEASE;
		}

		// Detectamos flancos de subida o bajada
		if (currB2State && !prevB2State)	// SUBIDA (0->1)
		{
			prevB2State = 1; // Actualizamos el valor posterior
			butChange   = 1; // Indicamos cambio de estado
			butReport = MOUSE_REPORT_BUTTON_1;
		}
		else if (!currB2State && prevB2State) // BAJADA (1->0)
		{
			prevB2State = 0; // Actualizamos el valor posterior
			butChange   = 1; // Indicamos cambio de estado
			butReport = MOUSE_REPORT_BUTTON_RELEASE;
		}

		// Solo mandamos reportes al host si ha habido cambios
		if(butChange || movChange)
		{
			// Indicamos estado de envio
			g_iMouseState = STATE_SENDING;
			uint32_t ui32Retcode = 0;
			uint8_t  bSuccess = 0;
			uint32_t numAtemp = 0;
				// Mandamos el reportaje continuamente si falla
			while(!bSuccess && numAtemp < 60000)
			{
				numAtemp++; // Numero de intentos
				// Mandamos el reporte
				ui32Retcode = USBDHIDMouseStateChange((void *) &g_sMouseDevice,
				xDistance,	// Desplazamiento de pixeles en el eje x
				yDistance,	// Desplazamiento de pixeles en el eje y
				butReport); // Estado de los botones

				// Si ha habido exito enviando el reporte
				if (ui32Retcode == MOUSE_SUCCESS)
				{
					// Esperamos a que el host reciba el reportaje si ha ido bien
					bSuccess = WaitForSendIdle(MAX_SEND_DELAY);

					// Se ha acabado el tiempo y no se ha puesto en IDLE?
					if (!bSuccess)
					{
						// Asumimos que el host se ha desconectado
						g_bConnected = false;
					}
				}
				else
				{
					// Error al mandar reporte ignoramos petcion e informamos
					// UARTprintf("No ha sido posible enviar reporte.\n");
					bSuccess = false;
				}
			}
			// Reseteamos las variables de cambios de estado
			butChange = 0;
			movChange = 0;
		}
	}
}
}
}
