#include <iostream>
#include <limits>
#include <cmath>
#include <iomanip>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define MAX_FLOAT numeric_limits<double>::max()

/* Filter Implementation */
Filter::Filter (double width)
  : container(), 
    filter_width(width),
    max_value(0), min_value(MAX_FLOAT) {}

void Filter::add_datapoint(const uint64_t timestamp, const double value)
{
  container.push_back(make_pair(timestamp, value));
  max_value = max(max_value, value);
  min_value = min(min_value, value);
}

double Filter::get_max()
{
  bool update_max = false;
  uint64_t curr = timestamp_ms();
  while (container.size() > 0 && container[0].first + filter_width < curr)
  {
    double old_value = container[0].second;
    if (fabs(old_value - max_value) < 1e6)
      update_max = true;
    container.pop_front();
  }
  if (container.size() == 0) return 0;
  if (update_max) {
    max_value = 0;
    for (auto const& item: container) max_value = max(max_value, item.second);
  }
  return max_value;
}

double Filter::get_min()
{
  bool update_min = false;
  uint64_t curr = timestamp_ms();

  while (container.size() > 0 && container[0].first + filter_width < curr)
  {
    double old_value = container[0].second;
    if (fabs(old_value - min_value) < 1e6)
      update_min = true;
    container.pop_front();
  }
  if (container.size() == 0) return 0;
  if (update_min) {
    min_value = MAX_FLOAT;
    for (auto const& item: container) min_value = min(min_value, item.second);
  }
  return min_value;
}


#define INIT_WIND 10        // in # of datagrams
#define PACKET_TIMEOUT 100  // in miliseconds

#define RTT_FILTER_WIDTH 1000    // in miliseconds
#define BW_FILTER_WIDTH 500      // in miliseconds

#define BRR_START_TIME 500
#define PACING_CYCLE 200


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), 
    cur_wind_( INIT_WIND ),
    rtt_filter ( Filter(RTT_FILTER_WIDTH) ),
    bw_filter ( Filter(BW_FILTER_WIDTH) ),
    delivered (0),
    delivery_map (),
    next_cycle_time ( BRR_START_TIME ),
    cycle_count (0)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  double bw   = bw_filter.get_max();
  double rtt  = rtt_filter.get_min();
  double bdp  = bw * rtt;
  uint64_t curr = timestamp_ms();

  // if (curr > next_cycle_time) {
  //   next_cycle_time = curr + (int)rtt;
  //   cycle_count ++;
  //   if (cycle_count % PACING_CYCLE == 0) {
  //     cur_wind_ = max(bdp * 1.25, 5.);
  //   } else if (cycle_count % PACING_CYCLE == 1) {
  //     cur_wind_ = bdp * 0.5;
  //   } else {
  //     cur_wind_ = bdp;
  //   }
  // }

  if (curr > BRR_START_TIME) {
    if (curr % PACING_CYCLE <= 20) {
      cur_wind_ = max(bdp * 1.25, 5.);
    } else if (curr % PACING_CYCLE <= 40) {
      cur_wind_ = bdp * 0.75;
    } else {
      cur_wind_ = bdp;
    }
  }

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

  delivery_map[sequence_number] = delivered;
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
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }

  /* Calculate RTT */
  int rtt = timestamp_ack_received - send_timestamp_acked;
  rtt_filter.add_datapoint(timestamp_ack_received, rtt);

  /* Calculate bandwidth */
  delivered ++;
  double delivery_rate = (double)(delivered - delivery_map[sequence_number_acked]) / rtt;
  delivery_map.erase (sequence_number_acked);
  bw_filter.add_datapoint(timestamp_ack_received, delivery_rate);
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return PACKET_TIMEOUT; /* timeout of one second */
}
