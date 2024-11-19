/* C Standard library */
#include <stdio.h>
#include <string.h>
/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"

/* Task */
#define STACKSIZE 2048
Char taskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
I2C_Handle i2cMPU;
I2C_Params i2cMPUParams;
// JTKJ: Teht�v� 3. Tilakoneen esittely
// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;

// JTKJ: Teht�v� 3. Valoisuuden globaali muuttuja
// JTKJ: Exercise 3. Global variable for ambient light
double ambientLight = -1000.0;
float ax, ay, az, gx, gy, gz;

// JTKJ: Teht�v� 1. Lis�� painonappien RTOS-muuttujat ja alustus
// JTKJ: Exercise 1. Add pins RTOS-variables and configuration here
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

// Pin configurations, with separate configuration for each pin
// The constant BOARD_BUTTON_0 corresponds to one of the buttons
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE // The configuration table is always terminated with this constant
};

// The constant Board_LED0 corresponds to one of the LEDs
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // The configuration table is always terminated with this constant
};



void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

    // JTKJ: Teht�v� 4. Lis�� UARTin alustus: 9600,8n1
    // JTKJ: Exercise 4. Setup here UART connection as 9600,8n1

    char input;
    char echo_msg[30];

    // UART-kirjaston asetukset
    UART_Handle uart;
    UART_Params uartParams;

    // Alustetaan sarjaliikenne
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);
      if (uart == NULL) {
         System_abort("Error opening the UART");
      }

      FILE *csvFile = fopen("sensor_data.csv", "w");
          if (csvFile == NULL) {
              System_abort("Error opening CSV file");
          }

          const char *header = "aika, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z\n";
              UART_write(uart, header, strlen(header));
              fprintf(csvFile, "%s", header);

    int time = 0;

    while (1) {

        // JTKJ: Teht�v� 3. Kun tila on oikea, tulosta sensoridata merkkijonossa debug-ikkunaan
        //       Muista tilamuutos
        // JTKJ: Exercise 3. Print out sensor data as string to debug window if the state is correct
        //       Remember to modify state



        // JTKJ: Teht�v� 4. L�het� sama merkkijono UARTilla
        // JTKJ: Exercise 4. Send the same sensor data string with UART

        // Vastaanotetaan 1 merkki kerrallaan input-muuttujaan
        //UART_read(uart, &input, 1);

              // Lähetetään merkkijono takaisin
        sprintf(echo_msg,"Received: %c\n",input);


              // Kohteliaasti nukkumaan sekunniksi
        //Task_sleep(1000000L / Clock_tickPeriod);

        // Just for sanity check for exercise, you can comment this out
        /*System_printf("uartTask\n");
        System_flush();*/

        if (programState == DATA_READY) {
            char sensorData[128];
            snprintf(sensorData, sizeof(sensorData),
                     "%d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                     time, ax, ay, az, gx, gy, gz);
            //System_printf(sensorData);
            System_flush();
            UART_write(uart, sensorData, strlen(sensorData));
            fprintf(csvFile, "%s", sensorData);
            fflush(csvFile);

            time += 1;
            if (time == 100) {
                fclose(csvFile);
                csvFile = NULL;  // Mark as closed
                System_printf("CSV file closed after 100 seconds.\n");
                System_flush();
                break;  // Exit the loop
            }



            programState = WAITING;
        }

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

void sensorTaskFxn(UArg arg0, UArg arg1) {


    //I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    //I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

        // MPU setup and calibration
        System_printf("MPU9250: Setup and calibration...\n");
        System_flush();

        mpu9250_setup(&i2cMPU);

        System_printf("MPU9250: Setup and calibration OK\n");
        System_flush();

    // Loop forever
    while (1) {

        // MPU ask data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

        programState = DATA_READY;

        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }

    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

Int main(void) {

    // Task variables
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle task;
    Task_Params taskParams;
    // Initialize board
    Board_initGeneral();

    
    // JTKJ: Teht�v� 2. Ota i2c-v�yl� k�ytt��n ohjelmassa
    // JTKJ: Exercise 2. Initialize i2c bus

    Board_initI2C();

    // JTKJ: Teht�v� 4. Ota UART k�ytt��n ohjelmassa
    // JTKJ: Exercise 4. Initialize UART

    Board_initUART();

    // JTKJ: Teht�v� 1. Ota painonappi ja ledi ohjelman k�ytt��n
    //       Muista rekister�id� keskeytyksen k�sittelij� painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button


    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }

    // Set the button pin’s interrupt handler to function buttonFxn
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (hMpuPin == NULL) {
            System_abort("Pin open failed!");
        }

    /* Task */
        Task_Params_init(&taskParams);
        taskParams.stackSize = STACKSIZE;
        taskParams.stack = &taskStack;
        task = Task_create((Task_FuncPtr)sensorTaskFxn, &taskParams, NULL);
        if (task == NULL) {
            System_abort("Task create failed!");
        }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
