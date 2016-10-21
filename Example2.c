/*********************************************************************
 *
 * Driver code for my UART implementation in VHDL.
 *
 * Uses register interface to populate and drain TX and RX FIFOs.
 *
 ********************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ZestSC1.h"

/*
 * Generic error handler function
 */
void ErrorHandler(const char *Function, 
                  ZESTSC1_HANDLE Handle,
                  ZESTSC1_STATUS Status,
                  const char *Msg)
{
    printf("**** Example2 - Function %s returned an error\n        \"%s\"\n\n", Function, Msg);
    exit(1);
}

/*
 * Main program - expects to be called with the number of bytes to 
 * receive, and reads them in bursts measured via the fifo_count RX register.
 */

int main(int argc, char **argv)
{
    unsigned long Count;
    unsigned long NumCards;
    unsigned long CardIDs[256];
    unsigned long SerialNumbers[256];
    ZESTSC1_FPGA_TYPE FPGATypes[256];
    ZESTSC1_HANDLE Handle;
    unsigned totalToRead;
    if (argc>=2)
        totalToRead = atoi(argv[1]);
    else
        totalToRead = 1;

    /*
     * Install the generic error handler
     */
    ZestSC1RegisterErrorHandler(ErrorHandler);

    /*
     * Request information about the system
     */
    ZestSC1CountCards(&NumCards, CardIDs, SerialNumbers, FPGATypes);
#ifdef VERBOSE
    printf("%lu available cards in the system\n", NumCards);
#endif
    if (NumCards==0)
    {
        printf("No cards in the system\n");
        exit(1);
    }

#ifdef VERBOSE
    for (Count=0; Count<NumCards; Count++)
    {
        printf("\t%lu : CardID = 0x%08lx, SerialNum = 0x%08lx, FPGAType = %d\n",
            Count, CardIDs[Count], SerialNumbers[Count], FPGATypes[Count]);
    }
#endif

    /*
     * Open the first card
     */
    ZestSC1OpenCard(CardIDs[0], &Handle);

    /*
     * Configure the FPGA
     */
    if (FPGATypes[0] == ZESTSC1_XC3S1000)
        ZestSC1ConfigureFromFile(Handle, "FPGA-VHDL/Example2.bit");
    else {
        puts("Unknown type of FPGA :-(");
        exit(1);
    }

#ifdef VERBOSE
    puts("Incoming data... Here we go:");
    fflush(stdout);
#endif
    Count = 0;
    while(Count<totalToRead) {
        unsigned char fifo_count = 0, fifo_empty, c;

        do {
            /* Wish the interrupt logic worked under Linux :-( */
            /* (sigh) Oh well, we can always poll              */
            ZestSC1ReadRegister(Handle, 0x2002, &fifo_empty);
        } while(fifo_empty);

        /* New data arrived in the RX FIFO! How much? */
        ZestSC1ReadRegister(Handle, 0x2010, &fifo_count);
        fifo_count++;

        /* Drain them */
        while (fifo_count--) {
            ZestSC1ReadRegister(Handle, 0x2000, &c);
            putchar(c);
            Count++;
            /* TX them back! */
            ZestSC1WriteRegister(Handle, 0x2000, c);
        }
        fflush(stdout);
    }
    /* Close the card - this used to sleep,
     * allowing the USB TX stream to be flushed out...
     */
    /* usleep(1e5); */
    ZestSC1CloseCard(Handle);

    return 0;
}
