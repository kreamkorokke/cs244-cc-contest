#include <iostream>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define INIT_WIND 10  // in # of datagrams
#define PACKET_TIMEOUT 60  // in miliseconds

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), cur_wind_((double)INIT_WIND)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size (double) is " << cur_wind_
   << "(double) and " << (unsigned int)cur_wind_
   << "(unsigned int)"
   << endl;
  }

  return (unsigned int)cur_wind_;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */

  uint64_t time_diff = timestamp_ack_received - send_timestamp_acked;
  
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;

   if (time_diff > timeout_ms()) {
     cerr << "At time " << timestamp_ack_received
       << " received ack that exceeded the timeout. Time diff: " << time_diff
       << endl;
   }
  }

  if (time_diff > timeout_ms()) {
    /* Timeout occured. Cut the window size in half */
    cur_wind_ /= 2;
    if ((unsigned int)cur_wind_ == 0) {
      cur_wind_ = 1.0;
    }
  } else {
    /* Increase by 1/w so that after an RTT, the window size
     * grows by one packet */
    cur_wind_ += 1/cur_wind_;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return PACKET_TIMEOUT; /* timeout of one second */
}
