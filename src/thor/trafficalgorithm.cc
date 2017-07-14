#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/trafficalgorithm.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace valhalla {
namespace thor {

// Default constructor
TrafficAlgorithm::TrafficAlgorithm()
    : AStarPathAlgorithm() {
}

// Destructor
TrafficAlgorithm::~TrafficAlgorithm() {
  Clear();
}

// Expand from a node in the forward direction
void TrafficAlgorithm::Expand(GraphReader& graphreader, const GraphId& node,
                              const EdgeLabel& pred, const uint32_t pred_idx,
                              const bool from_transition) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // TODO - manage traffic data for this tile?
//  std::vector<uint8_t>& speeds = GetRealTimeSpeeds(node.tileid(), graphreader);

  // Expand from end node in forward direction.
  uint32_t shortcuts = 0;
  GraphId edgeid = { node.tileid(), node.level(), nodeinfo->edge_index() };
  const DirectedEdge* directededge = tile->directededge(edgeid);
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid) {
    // Skip shortcut edges (no traffic information is applied)
    if (directededge->is_shortcut()) {
      continue;
    }
    // Handle transition edges - expand from the end node of the transition
    // (unless this is called from a transition).
    if (directededge->trans_up()) {
      if (!from_transition) {
//        hierarchy_limits_[node.level()].up_transition_count++;
        Expand(graphreader, directededge->endnode(), pred, pred_idx, true);
      }
      continue;
    }
    if (directededge->trans_down()) {
      if (!from_transition/* &&
          !hierarchy_limits_[directededge->endnode().level()].StopExpanding()*/) {
        Expand(graphreader, directededge->endnode(), pred, pred_idx, true);
      }
      continue;
    }

    // Get the current set. Skip this edge if permanently labeled (best
    // path already found to this directed edge).
    EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
    if (edgestatus.set() == EdgeSet::kPermanent) {
      continue;
    }

    // Skip this edge if no access is allowed (based on costing method)
    // or if a complex restriction prevents transition onto this edge.
    if (!costing_->Allowed(directededge, pred, tile, edgeid) ||
         costing_->Restricted(directededge, pred, edgelabels_, tile,
                                     edgeid, true)) {
      continue;
    }

    // TODO - do we want to add a traffic costing method in sif or do
    // we do traffic influence here and fall back to SIF costing for the
    // specified mode (e.g. would allow truck, HOV, bus, etc to be influenced
    // by traffic
    /**
    Cost edge_cost;
    Cost tc = costing_->TransitionCost(directededge, nodeinfo, pred);
    if (speeds.size() == 0 || speeds[edgeid.id()] == 0) {
      edge_cost = costing_->EdgeCost(directededge);
    } else {
    // Traffic exists for this edge
    float sec = directededge->length() * (kSecPerHour * 0.001f) /
           static_cast<float>(speeds[edgeid.id()]);
    edge_cost = { sec, sec };

    // For now reduce transition cost by half...thought is that traffic
    // will account for some of the transition cost
    tc.cost *= 0.5f;
    tc.secs *= 0.5f;
    }
    */

    // Get cost. Separate out transition cost.
    Cost tc = costing_->TransitionCost(directededge, nodeinfo, pred);
    Cost newcost = pred.cost() + tc + costing_->EdgeCost(directededge);

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      newcost -= p->second;
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (edgestatus.set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[edgestatus.index()];
      if (newcost.cost <  lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(edgestatus.index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, tc);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;
    if (p == destinations_.end()) {
      const GraphTile* t2 = directededge->leaves_tile() ?
          graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += astarheuristic_.Get(
                  t2->node(directededge->endnode())->latlng(), dist);
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_.size();
    adjacencylist_->add(idx, sortcost);
    edgestatus_->Set(edgeid, EdgeSet::kTemporary, idx);
    edgelabels_.emplace_back(pred_idx, edgeid, directededge,
                  newcost, sortcost, dist, mode_, 0);
  }
}

// Calculate best path. This method is single mode, not time-dependent.
std::vector<PathInfo> TrafficAlgorithm::GetBestPath(PathLocation& origin,
             PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing_ = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.edges.front().projected, destination.edges.front().projected, costing_);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination, costing_);
  SetOrigin(graphreader, origin, destination, costing_);

  // Minimum distance to the destinaton and count of iterations with no
  // convergence towards destination. Used to break out of algorithm when
  // a destination cannot be reached.
  uint32_t nc = 0;
  float mindist = astarheuristic_.GetDistance(origin.edges.front().projected);

  // TODO - get time and add method to iterate through time

  // Find shortest path
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
      return { };
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(predindex);
        }
      } else {
        return FormPath(predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 500000) {
      LOG_ERROR("No convergence to destination after = " +
                           std::to_string(edgelabels_.size()));
      return {};
    }

    // Expand from the end node in forward direction.
    Expand(graphreader, pred.endnode(), pred, predindex, false);
  }
  return {};      // Should never get here
}

std::vector<uint8_t>& TrafficAlgorithm::GetRealTimeSpeeds(const uint32_t tileid,
                               GraphReader& graphreader) {
  // Check if this tile has real-time speeds
  auto rts = real_time_speeds_.find(tileid);
  if (rts == real_time_speeds_.end()) {
    // Try to load the speeds file
    std::ifstream rtsfile;
    std::string traffic_dir = graphreader.tile_dir() + "/traffic/";
    std::string fname = traffic_dir + std::to_string(tileid) + ".spd";
    rtsfile.open(fname, std::ios::binary | std::ios::in | std::ios::ate);
    if (rtsfile.is_open()) {
      uint32_t count = rtsfile.tellg();
      LOG_INFO("Load real time speeds: count = " + std::to_string(count));
      rtsfile.seekg(0, rtsfile.beg);
      std::vector<uint8_t> spds(count);
      rtsfile.read((char*)(&spds.front()), count);
      rtsfile.close();
      real_time_speeds_[tileid] = spds;
      return real_time_speeds_[tileid];
    } else {
      return empty_speeds_;
    }
  } else {
    return rts->second;
  }
}


}
}
