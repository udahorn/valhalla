#ifndef VALHALLA_BALDR_HISTORICAL_SPEED_TILE_H_
#define VALHALLA_BALDR_HISTORICAL_SPEED_TILE_H_

#include <cstdint>

#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

// Example size of a tile with 10000 OSMLR segments, with an average of 1.5
// transitions per segment:
//    Reference Speeds:      10000
//    Historical Speeds:   1680000
//    Transition Times:     870000
//    Total:               2560000   2.5 MB
//
// To support OSMLR Level 2 (speeds but no transition times):
//    100000 OSMLR segments
//    Reference Speeds:      100000
//    Historical Speeds:   16800000
//    Total:               16900000 16.9 MB

struct HistoricalSpeedHeader {
  GraphId  graphid_;            // GraphId of the tile
  uint32_t segment_count_;      // Number of OSMLR segments
  uint32_t transition_count_;   // Number of transition records

  // TODO - some indication of time frame?
};

// 58 bytes per transition
struct TransitionTime {
  uint64_t segment_id_;
  GraphId next_segment_;
  uint8_t seconds_[midgard::kHoursPerWeek];   // Do we want to make this a variable size?
};

using transition_pair_t = std::pair<GraphId, uint8_t[midgard::kHoursPerWeek]>;

/**
 * Historical speed information for a tile within the Tiled Hierarchical Graph.
 */
class HistoricalSpeedTile {
 public:

  /**
   * Constructor
   */
  HistoricalSpeedTile();

  /**
   * Constructor given a GraphId. Reads the graph tile from file
   * into memory.
   * @param  tile_dir   Tile directory.
   * @param  graphid    GraphId (tileid and level)
   */
  HistoricalSpeedTile(const std::string& tile_dir, const GraphId& graphid);

  /**
   * Destructor
   */
  virtual ~HistoricalSpeedTile();

  /**
   * Gets a pointer to the historical speed tile header.
   * @return  Returns the header for the graph tile.
   */
  const HistoricalSpeedHeader* header() const {
    return header_;
  }

  /**
   * Get the reference speed given the OSMLR segment Id.
   * @param  osmlr_id  OSMLR segment Id.
   * @return Returns the reference speed in kph.
   */
  uint32_t reference_speed(const GraphId& osmlr_id) const {
    // TODO - verify tile?
    return (osmlr_id < header_->segment_count_) ? reference_speeds_[osmlr_id] : 0;
  }

  /**
   * Get the historical speed given the OSMLR segment Id and hour of the week.
   * @param  osmlr_id  OSMLR segment Id.
   * @param  hour      Hour of the week. TODO - do we pass in more standard time struct?
   * @return Returns the reference speed in kph.
   */
  uint32_t reference_speed(const GraphId& osmlr_id, const uint32_t hour) const {
    // TODO - verify tile?
    return (osmlr_id < header_->segment_count_) ?
        *(historical_speeds_ + (osmlr_id * midgard::kHoursPerWeek) + hour) : 0;
  }

  // TODO - transition time support.

 protected:

  // Header information for the tile
  HistoricalSpeedHeader* header_;

  // List of reference speeds. A single reference speed per segment.
  uint8_t* reference_speeds_;

  // List of historical speeds. Number of segments * number of entries.
  uint8_t* historical_speeds_;

  // Map of transition times (implementation TBD - read in at tile load?)
  std::unordered_map<uint32_t, transition_pair_t> transition_times_;
};

}
}

#endif  // VALHALLA_BALDR_HISTORICAL_SPEED_TILE_H_
