#include <iostream>
#include <limits>
#include <cmath>
#include <iomanip>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/** Filter Implementation **
A filter can track the max or min value within a timerange [T-W, T], where
T is the current time and W is the width of the filter. 
*/

#define MAX_FLOAT numeric_limits<double>::max()

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


/** Averager Implementation **
An averager tracks a moving average of data points. The old values are exponentially
damped by a decay factor.
*/
Averager::Averager ()
  : decay_factor (-1),
    average (0),
    counter (0) {}

Averager::Averager (double decay)
  : decay_factor (decay),
    average (0),
    counter (0) {}

void Averager::add_datapoint(const double value) 
{
    if (decay_factor < 0) {
        average = (average * counter + value) / (counter + 1);
    } else {
        average = average * decay_factor + value * (1 - decay_factor);
    }
    counter ++;
}

double Averager::get_avg()
{
    return average;
}


/** Congestion Controller Implementation **/


#define INIT_WIND 10        // in # of datagrams
#define PACKET_TIMEOUT 100  // in miliseconds

#define RTT_FILTER_WIDTH 10000   // in miliseconds
#define BW_FILTER_WIDTH 100      // in miliseconds

#define RTT_DECAY_FACTOR 0.8

#define BBR_START_TIME 500


/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), 
    cur_wind_( INIT_WIND ),
    rtt_averager (Averager(RTT_DECAY_FACTOR)),
    rttprop_filter (Filter(RTT_FILTER_WIDTH)),
    bw_filter (Filter(BW_FILTER_WIDTH)),
    delivered (0),
    cache ()
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Update window size */
  uint64_t curr = timestamp_ms();

  double rtt_avg = rtt_averager.get_avg();
  double rtt_thres = rttprop_filter.get_min() * 1.3;
  double bw = bw_filter.get_max();
  double bdp = bw * rtt_thres;

  if (curr > BBR_START_TIME) {
    if (rtt_avg < rtt_thres) {
        cur_wind_ = (int)(bdp * 1.15 + .5);
      } else {
        cur_wind_ = (int)(bdp * 0.85 + .5);
      }
  }

  /* Prevent window size from dropping to zero */
  if (cur_wind_ <= 1) cur_wind_ = 1;


  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size (int) is " << cur_wind_
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

  cache[sequence_number] = delivered;
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
  rtt_averager.add_datapoint(rtt);
  rttprop_filter.add_datapoint(timestamp_ack_received, rtt);

  /* Calculate bandwidth */
  delivered ++;
  double delivery_rate = (double)(delivered - cache[sequence_number_acked]) / rtt;
  cache.erase (sequence_number_acked);
  bw_filter.add_datapoint(timestamp_ack_received, delivery_rate);
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return PACKET_TIMEOUT; /* timeout of one second */
}
