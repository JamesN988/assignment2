#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/

struct sender_packet {
  struct pkt packet;
  bool acked;
  bool sent;
};

static struct sender_packet A_buffer[SEQSPACE];
static int A_base = 0;
static int A_nextseq = 0;

int ComputeChecksum(struct pkt packet)
{
  int checksum = packet.seqnum + packet.acknum;
  for ( int i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
    return packet.checksum != ComputeChecksum(packet);
}

void A_output(struct msg message)
{
  if (((A_nextseq - A_base + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
    struct pkt pkt;
    pkt.seqnum = A_nextseq;
    pkt.acknum = NOTINUSE; 

  for (int i=0; i < 20; i++)
    pkt.payload[i] = message.data[i];

  pkt.checksum = ComputeChecksum(pkt);

  A_buffer[A_nextseq].packet = pkt;
  A_buffer[A_nextseq].acked = false;
  A_buffer[A_nextseq].sent = true;

  tolayer3(A,pkt);
  starttimer(A,RTT);

  if (TRACE > 1)
    printf("A_output: Sent packet %d\n", pkt.seqnum);

  A_nextseq = (A_nextseq + 1) % SEQSPACE;
    } else {
        if (TRACE > 0)
            printf("A_output: Window full, message dropped\n");
        window_full++;
    }
}

void A_input(struct pkt packet)
{
  if (!IsCorrupted(packet)) {
    int acknum = packet.acknum;

    if (TRACE > 1)
      printf("A_input: ACK %d is received\n",acknum);

    /* Mark packet as acknowledged */
    if (A_buffer[acknum].sent && !A_buffer[acknum].acked) {
      A_buffer[acknum].acked = true;
      new_ACKs++;
      total_ACKs_received++;
    }

    // Slide window if base packet was ACKed
    while (A_buffer[A_base].acked) {
      A_buffer[A_base].sent = false;
      A_base = (A_base + 1) % SEQSPACE;
  }

  // Restart timer for the next unACKed packet
  stoptimer(A);
  for (int i = 0; i < WINDOWSIZE; i++) {
      int idx = (A_base + i) % SEQSPACE;
      if (A_buffer[idx].sent && !A_buffer[idx].acked) {
          starttimer(A, RTT);
          break;
      }
  }
} else {
  if (TRACE > 0)
      printf("A_input: Corrupted ACK received, ignored\n");
}

}

void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("A_timerinterrupt: Retransmitting unACKed packets\n");

  for(int i=0; i<window_full; i++) {
    int idx = (A_base + i) % SEQSPACE;
    if (A_buffer[idx].sent && !A_buffer[idx].acked) {
      tolayer3(A,A_buffer[idx].packet);
      packets_resent++;
      if (i==0)
        starttimer(A,RTT);
    }
  }
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  for (int i = 0; i < SEQSPACE; i++)
    A_buffer[i].acked = A_buffer[i].sent = false;

  A_base = 0; 
  A_nextseq = 0;
}



/********* Receiver (B)  variables and procedures ************/

static struct pkt B_buffer[SEQSPACE];      // Buffer for received packets
static bool B_received[SEQSPACE];          // Which packets have been received
static int B_expected = 0;                 // Base of receive window


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  if  (!IsCorrupted(packet)) {
    int seq = packet.seqnum;

    // Check if within receive window
        if (((seq - B_expected + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
            // Accept and buffer if not already received
            if (!B_received[seq]) {
                B_buffer[seq] = packet;
                B_received[seq] = true;
                packets_received++;
            }

            // Deliver all in-order packets from B_expected onward
            while (B_received[B_expected]) {
                tolayer5(B, B_buffer[B_expected].payload);
                B_received[B_expected] = false;
                B_expected = (B_expected + 1) % SEQSPACE;
            }
        }

        // Send individual ACK
        struct pkt ackpkt;
        ackpkt.seqnum = 0;  // Not used
        ackpkt.acknum = seq;
        for (int i = 0; i < 20; i++)
            ackpkt.payload[i] = 0;
        ackpkt.checksum = ComputeChecksum(ackpkt);
        tolayer3(B, ackpkt);

        if (TRACE > 1)
            printf("B_input: ACK %d sent\n", seq);
    } else {
        if (TRACE > 0)
            printf("B_input: Corrupted packet received, ignored\n");
    }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
    int i;
    for (i = 0; i < SEQSPACE; i++) {
        B_received[i] = 0;
    }
    B_expected = 0;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

