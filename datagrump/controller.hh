#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <deque>
#include <map>

class Filter
{
private:
  std::deque <std::pair<uint64_t, double>> container;
  int filter_width;
  double max_value;
  double min_value;

public:
  Filter(double width);
  void add_datapoint(const uint64_t timestamp, const double value);
  double get_max();
  double get_min();
};


/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */

  /* Current window size in # of datagramss */
  double cur_wind_;

  Filter rtt_filter;
  Filter bw_filter;

  int delivered;
  std::map <uint64_t, int> delivery_map;

  uint64_t next_cycle_time;
  int cycle_count;

public:
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );
};

#endif
