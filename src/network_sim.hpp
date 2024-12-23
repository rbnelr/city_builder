#pragma once
#include "common.hpp"
#include "network.hpp"

class App;

namespace network {
struct Network;

inline Bezier3 Node::calc_curve (Segment* seg0, Segment* seg1, float2 shiftXZ_0, float2 shiftXZ_1) {
	auto i0 = seg0->get_end_info(this, shiftXZ_0);
	auto i1 = seg1->get_end_info(this, shiftXZ_1);

	float3 ctrl_in, ctrl_out;
	float2 point;
	// Find straight line intersection of in/out lanes with their tangents
	if (line_line_intersect((float2)i0.pos, (float2)i0.forw, (float2)i1.pos, (float2)i1.forw, &point)) {
		ctrl_in  = float3(point, i0.pos.z);
		ctrl_out = float3(point, i1.pos.z);
	}
	// Come up with seperate control points TODO: how reasonable is this?
	else {
		float dist = distance(i0.pos, i1.pos) * 0.5f;
		ctrl_in  = i0.pos + float3((float2)i0.forw, 0) * dist;
		ctrl_out = i1.pos + float3((float2)i1.forw, 0) * dist;
	}

	// NOTE: for quarter circle turns k=0.5539 would result in almost exactly a quarter circle!
	// https://pomax.github.io/bezierinfo/#circles_cubic
	// but turns that are sharper in the middle are more realistic, but we could make this customizable?
	float k = 0.6667f;

	Bezier3 bez;
	bez.a = i0.pos;
	bez.b = lerp(i0.pos, ctrl_in , k);
	bez.c = lerp(i1.pos, ctrl_out, k);
	bez.d = i1.pos;
	return bez;
}
inline Bezier3 Node::calc_curve (SegLane& in, SegLane& out) {
	// calc_curve works with shifts 
	auto& a0 = in.get_asset();
	auto& a1 = out.get_asset();
	float shift0 = a0.direction == LaneDir::FORWARD ? a0.shift : -a0.shift;
	float shift1 = a1.direction == LaneDir::FORWARD ? -a1.shift : a1.shift;

	return calc_curve(in.seg, out.seg, float2(shift0, ROAD_Z), float2(shift1, ROAD_Z));
}

inline ParkingSpot* find_building_parking (Building* dest) {
	for (auto& spot : dest->parking) {
		if (spot.avail())
			return &spot;
	}
	return nullptr;
}
inline ParkingSpot* find_street_parking (Segment* seg) {
	for (auto& spot : seg->parking.spots) {
		if (spot.avail())
			return &spot;
	}
	return nullptr;
}
inline ParkingSpot* find_parking_near (Building* dest) {
	auto* spot = find_building_parking(dest);
	if (spot) return spot;

	return find_street_parking(dest->connected_segment);
}

// Stores the path from pathfinding
// Is a Sequence of Motions that a vehicle performs to drive along a path acting as a state machine
// step() should be called whenever the vehicle has performed the Motion represented by the current Motion
// For pedestrians this needs to be reworked
// Pathfinding will later pathfind walking/driving/transit in one go, so there can't be a vehicle specific path
//  but vehicles will still need to be able to ask for the next motion
class Path {
public:
	enum MotionType {
		STARTUP, SHUTDOWN,
		START, END,
		SEGMENT, NODE
	};

	struct Motion {
		// current path sequence number (complicated, ideally this could be a generator function)
		// 0: start building -> road segment
		// 1: first segment, 2: first node, 3: next segment...
		// n-1: road segment -> end building
		int idx = -1;
		
		MotionType motion;

		float end_t = 1.0f;
		float next_start_t = 0.0f;
		Bezier3 bezier;

		// Should this include slowdown during curves (and if so should it smoothly ramp down the speed somehow?)?
		float cur_speedlim; // speed limit on current lane
		float next_speedlim; // speed limit on next lane

		// if vehicle front either in incoming lane before node or in node:
		// cur_lane: incoming, next_lane: outgoing lane
		SegLane cur_lane;
		SegLane next_lane;

		// if vehicle front either in incoming lane before node or in node, else null
		Node* get_cur_node () const {
			if (cur_lane && next_lane)
				return Node::between(cur_lane.seg, next_lane.seg);
			return nullptr;
		}

		// Needed for correct Segment/Node updates
		LaneVehicles* cur_vehicles = nullptr;

		bool valid () {
			return idx >= 0;
		}
	};

	void mem_use (MemUse& mem) {
		mem.add("Path", sizeof(*this));
		for (auto& i : path) mem.add("Path::path[]", sizeof(i));
	}
	
	// Make Endpoint/EndCurve a seperate class that handles everything including Pathfinding forw/backw flags?
	struct Endpoint {
		Building* building = nullptr;
		ParkingSpot* parking = nullptr;

		struct Curve {
			Bezier3 bez;
			float lane_t;

			struct Lane {
				SegLane const& lane;
				bool dir;
			};
	
			static float get_lane_t (SegLane const& lane, float lane_t) {
				float t = lane.get_asset().direction == LaneDir::FORWARD ? lane_t : 1.0f - lane_t;
				assert(t >= 0.0f && t <= 1.0f);
				return t;
			}

			static Curve calc_bezier (Lane const& lane, float3 pos, float3 ctrl, float lane_t) {
				auto lane_bez = lane.lane._bezier();
				float len = lane_bez.approx_len(4);
				float ctrl_t = min(3 / len, 0.5f); // control point 3m from actual nearest point

				float t0, t1;
				if (lane.dir == false) {
					t1 = clamp(get_lane_t(lane.lane, lane_t), 0.01f, 0.99f - ctrl_t); // limit such that t0 still in range
					t0 = t1 + ctrl_t;
				} else {
					t1 = clamp(get_lane_t(lane.lane, lane_t), 0.01f + ctrl_t, 0.99f); // limit such that t0 still in range
					t0 = t1 - ctrl_t;
				}
				assert(t0 > 0.0f && t0 < 1.0f);
				assert(t1 > 0.0f && t1 < 1.0f);
	
				float3 lane_ctrl = lane_bez.eval(t1).pos;
				float3 lane_pos  = lane_bez.eval(t0).pos;
		
				if (lane.dir == false)
					return { Bezier3(pos, ctrl, lane_ctrl, lane_pos), t0 };
				else
					return { Bezier3(lane_pos, lane_ctrl, ctrl, pos), t0 };
			}
	
			static Curve from_nearest_point_on_sidewalk (Lane const& lane, Segment* seg, float3 pos) {
				float t;
				seg->distance_to_point(pos, &t);
		
				auto bez = seg->bezier().eval(t);
				auto right = bez.dirs().right;

				float right_dist = dot(right, pos - bez.pos);
				float offset1 = right_dist > 0.0f ? seg->asset->sidewalkR : seg->asset->sidewalkL;
				float offset2 = right_dist > 0.0f ? seg->asset->sidewalkR+1 : seg->asset->sidewalkL-1;

				float3 end_pos = bez.pos + right * offset2;
				float3 ctrl    = bez.pos + right * offset1;

				return calc_bezier(lane, end_pos, ctrl, t);
			}

			//static NavEndPath building_viz (Lane const& lane, Building* build) {
			//	float t;
			//	float3 ctrl = PosRot(build->pos, build->rot).local(float3(-5, 0, 0));
			//	build->connected_segment->distance_to_point(build->pos, &t);
			//	return calc_bezier(lane, build->pos, ctrl, t);
			//}
			static Curve building_front (Lane const& lane, Building* build) { // for vehicles despawning "pocket cars"
				return from_nearest_point_on_sidewalk(lane, build->connected_segment, build->pos);
			}
			static Curve building_parking (Lane const& lane, Building* build, ParkingSpot* parking) {
				float3 pos = parking->vehicle_front_pos().pos;
				float3 ctrl = parking->front_enter_ctrl();
				float t;
				build->connected_segment->distance_to_point(ctrl, &t);
				return calc_bezier(lane, pos, ctrl, t);
			}
			static Curve street_parking (Lane const& lane, Building* build, ParkingSpot* parking) {
				float3 pos = parking->vehicle_front_pos().pos;
				float3 ctrl = lane.dir ? parking->side_enter_ctrl() : parking->side_exit_ctrl();
				float t;
				build->connected_segment->distance_to_point(ctrl, &t);
				return calc_bezier(lane, pos, ctrl, t);
			}
		
			static Curve calc (Endpoint const& ep, Curve::Lane const& lane, bool visualize=false) {
				//if (visualize)
				//	return NavEndCurve::building_viz(lane, building);

				if (ep.parking) {
					bool is_building_parking = kiss::contains(ep.building->parking, ep.parking,
						[] (ParkingSpot const& l, ParkingSpot const* r) { return &l == r; });
					if (is_building_parking)
						return Curve::building_parking(lane, ep.building, ep.parking);
					else
						return Curve::street_parking(lane, ep.building, ep.parking);
				}
				else {
					return Curve::building_front(lane, ep.building);
				}
			}
		};
	};

	Endpoint start;
	Endpoint dest;

	std::vector<Segment*> path;

	// start and destination getters implemented by Trip
	Endpoint::Curve get_trip_start (SegLane lane) {
		return Endpoint::Curve::calc(start, {lane, false});
	}
	// false: parking spot gets reserved on call, this call only happens just before dest is reached
	// visualize=true to avoid instantly reserving parking spot
	Endpoint::Curve get_trip_dest (SegLane lane, Vehicle& veh, bool visualize=false) {
		// find and reserve parking if not reserved yet, and do not reserve if just visualizing future path
		if (!dest.parking && !visualize) {
			dest.parking = find_parking_near(dest.building);
			if (dest.parking) {
				dest.parking->reserve(&veh);
			}
		}
		return Endpoint::Curve::calc(dest, {lane, true});
	}
	
	// HACK: to fix problem with node vehicle tracking
	void _clear_nodes (Vehicle& veh) {
		int num_seg = (int)path.size();
		
		for (int i=0; i<num_seg-1; ++i) {
			auto* node = Node::between(path[i], path[i+1]);
			node->vehicles.test.try_remove(&veh);
		}
	}
	void _dtor (Vehicle& veh) {
		_clear_nodes(veh);

		// need to unreserve if deleted vehicle with trip etc.
		if (dest.parking && dest.parking->reserved)
			dest.parking->unreserve(&veh);
	}
	
	struct PathEnd {
		Segment* seg;
		// Able to pathfind while restricting the starting directions for example for repathing
		bool forw = true, backw = true;
	};
	static bool pathfind (Network& net, PathEnd start, PathEnd dest, std::vector<Segment*>* result_path);
	
	static std::optional<Path> begin (Network& net, Endpoint start, Endpoint dest);
	bool repath (Network& net, Endpoint new_dest, Vehicle& veh);
	
	void visualize (OverlayDraw& overlay, Network& net, Vehicle& veh, bool skip_next_node, lrgba col=lrgba(1,1,0,0.75f));
	
	SegLane pick_lane (Network& net, Random& rand, int seg_i, SegLane prev_lane) const;

	Motion get_motion (Network& net, int idx, Motion* prev_state, Vehicle& veh, bool visualize);
	
	void step_vehicle (Network& net, Vehicle& veh);

	void begin_vehicle_trip (Network& net, Vehicle& veh);
	void cancel_vehicle_trip (Vehicle& veh);
	void finish_vehicle_trip (Vehicle& veh);
};

class SimVehicle {
public:
	void mem_use (MemUse& mem) {
		mem.add("SimVehicle", sizeof(*this));
	}

	static constexpr float STARTUP_DURATION = 5;

	Path* path; // TODO: could potentially eliminated

	Path::Motion mot;
	
	// [0,1] bezier parameter for current segment/node curve
	// or parking/unparking startup timer
	float mot_t = 0;

	float brake = 1; // set by controlled conflict logic, to brake smoothly
	float speed = 0; // worldspace speed controlled by acceleration and brake

	// speed (delta position) / delta beizer t
	// INF to force no movement on initial tick (rather than div by 0)
	// set after timestep based on current bezier eval, to approx correct worldspace step size along bezier in next tick
	float bez_speed = INF;

//// Movement sim variables for visuals
	float3 front_pos; // car front
	float3 rear_pos; // car rear

	// another velocity parameter, this time for the center of the car, to implement suspension
	// TODO: this should not exist, OR be the only velocity paramter
	float3 center_vel = 0;
	// suspension (ie. car wobble) angles based on car acceleration on forward and sideways axes (computed seperately)
	float3 suspension_ang = 0; // angle in radians, X: sideways (rotation on local X), Y: forwards
	float3 suspension_ang_vel = 0; // angular velocity in radians
	
	// curvature, ie. 1/turn_radius, positive means left
	float turn_curv = 0;
	float wheel_roll = 0;

	float blinker = 0;
	float blinker_timer = 0; // could get eliminated (a fixed number of blinker timers indexed using vehicle id hash)
	float brake_light = 0;

	void _dtor (Vehicle& veh) {
		if (mot.cur_vehicles) mot.cur_vehicles->list.try_remove(&veh);
	}
	
	static PosRot get_init_pos (Bezier3 const& bez, VehicleAsset* asset) {
		auto pos = bez.eval(0).pos;
		auto forw = normalizesafe(bez.eval(0.001f).pos - pos);

		return PosRot{ pos, angle2d(forw) };
	}
	static PosRot get_init_pos (ParkingSpot* parking) {
		return parking->vehicle_front_pos();
	}

	void init_pos (PosRot pos, VehicleAsset* asset) {
		front_pos = pos.pos;
		rear_pos = pos.pos - (rotate3_Z(pos.ang) * float3(1,0,0)) * asset->length();
	}

	float3 _center () { return (front_pos + rear_pos)*0.5; };

	PosRot _calc_pos () {
		float ang = angle2d((float2)front_pos - (float2)rear_pos);
		return PosRot{ _center(), ang };
	}
	
	bool update_blinker (float rand_num, float dt) {
		constexpr float blinker_freq_min = 1.6f;
		constexpr float blinker_freq_max = 1.2f;

		blinker_timer += dt * lerp(blinker_freq_min, blinker_freq_max, rand_num);
		blinker_timer = fmodf(blinker_timer, 1.0f);
		return blinker_timer > 0.5f;
	}
};

class Vehicle {
public:
	VehicleAsset* asset;
	lrgb tint_col;
	float agressiveness;

	IVehicleOwner* owner;

	// When simulated:
	//  possibly reserved parking
	// When inactive:
	//  parked at building, or on street
	//  or null: invisible vehicle, probably pocket car
	ParkingSpot* parking = nullptr;

	std::unique_ptr<SimVehicle> sim = nullptr;

	Path* try_get_path () {
		return sim ? sim->path : nullptr;
	}

	std::optional<PosRot> calc_pos () {
		if (sim)     return sim->_calc_pos();
		if (parking) return parking->vehicle_center_pos(this);
		//if (owner)   return PosRot{ owner->calc_pos(), 0 };
		return {};
	}
	
	bool selectable () {
		return sim || parking != nullptr;
	}
	std::optional<SelCircle> get_sel_shape () {
		auto pos = calc_pos();
		if (pos)
			return SelCircle{ pos->pos, asset->length()*0.5f, lrgb(0.04f, 1, 0.04f) };
		return {};
	}
	
	void begin_update () {
		sim->brake = 1;
	}
	bool update (Path& path, App& app, Network& net, Metrics::Var& met, float dt);


	Vehicle (IVehicleOwner* owner, VehicleAsset* asset, lrgb tint_col, float agressiveness):
		owner{owner}, asset{asset}, tint_col{tint_col}, agressiveness{agressiveness} {}
	Vehicle (Vehicle&& v): // Allow static constructor (create_random_vehicle) and inheritance at the same time
		owner{v.owner}, asset{v.asset}, tint_col{v.tint_col}, agressiveness{v.agressiveness} {}
	
	void mem_use (MemUse& mem) {
		mem.add("Vehicle", sizeof(*this));
		if (sim) mem.add(*sim);
	}

	virtual ~Vehicle () {
		if (parking) parking->clear(this);
	}


	static VehicleAsset* pick_random_asset (Assets& assets, Random& rand) {
		auto rand_car = WeightedChoice(assets.vehicles.begin(), assets.vehicles.end(),
			[] (AssetPtr<VehicleAsset> const& car) { return car->spawn_weight; });

		return rand_car.get_random(rand)->get();
	}
	static lrgb pick_random_color (Random& rand, VehicleAsset* asset) {
		bool van = asset->mesh_filename == "vehicles/van.fbx";
		bool bus = asset->mesh_filename == "vehicles/bus.fbx";

		lrgb col;
		// 50% of cars (100% of busses) are colorful
		if (rand.chance(0.5f) || bus) {
			col = hsv2rgb(rand.uniformf(), 1.0f, 0.8f); // debug-like colorful colors
		}
		// other cars get black and white colors
		else {
			lrgb std_colors[] = {
				lrgb(0,0,0), // black
				lrgb(0,0,0), // 2nd black to make it more common
				lrgb(0.1f,0.1f,0.1f), // grey
				lrgb(0.5f,0.5f,0.55f), // silver?
				lrgb(1,1,1), // white
				lrgb(0.95f,0.1f,0.1f), // ?
			};
			col = std_colors[rand.uniformi(0, ARRLEN(std_colors))];
		}

		// most transporters are white
		if (van && rand.chance(0.8f)) {
			col = lrgb(1,1,1);
		}

		return col;
	}
	static float get_random_aggressiveness (Random& rand) {
		float agressiveness_deviation = 0.15f;
		return rand.normalf(agressiveness_deviation, 0.0f);
	}
	float aggressiveness_topspeed_accel_mul () const {
		return clamp(1.1f + agressiveness, 0.7f, 1.5f);
	}

	static Vehicle create_random_vehicle (IVehicleOwner* owner, Assets& assets, Random& rand) {
		auto* asset = pick_random_asset(assets, rand);
		auto tint_col = pick_random_color(rand, asset);
		auto agressiveness = get_random_aggressiveness(rand);
		return { owner, asset, tint_col, agressiveness };
	}
};

class PersonTrip {
public:
	void mem_use (MemUse& mem) {
		//mem.add("PersonTrip", sizeof(*this));
		path.mem_use(mem);
	}
	
	Path path;
	
	static bool begin_trip (Person& person, Network& net, Entities& entities, Random& rand);

	void cancel_trip (Person& person);
	void finish_trip (Person& person);

	static void update (App& app, Person& person, Network& net, Entities& entities, Metrics::Var& met, Random& rand, float dt);
};

class DebugVehicle : public Vehicle {
friend class DebugVehicles;

	Path path;

	DebugVehicle (IVehicleOwner* owner, Assets& assets):
		Vehicle{ Vehicle::create_random_vehicle(owner, assets, random) } {

	}
};
class DebugVehicles : IVehicleOwner {
public:
	std::vector<std::unique_ptr<DebugVehicle>> vehicles;
	
	std::unique_ptr<DebugVehicle> next_vehicle = nullptr;
	DebugVehicle* preview_veh = nullptr;
	
	bool place_vehicle = false;
	bool control_vehicle = false;
	
	void remove_vehicle (Vehicle* vehicle) override {
		remove_first(vehicles, vehicle, [] (std::unique_ptr<DebugVehicle> const& l, Vehicle* r) { return l.get() == r; });
	}

	void imgui () {
		if (ImGui::Button("Clear All")) {
			vehicles.clear();
			vehicles.shrink_to_fit();
		}
	}
	void update_interact (Input& input, Assets& assets, sel_ptr hover, std::optional<PosRot> hover_pos) {
		
		if (!next_vehicle) {
			// Create a random vehicle when using DebugVehicles tool,
			// but keep vehicle or else it flashes different random vehicles every frame
			next_vehicle = std::make_unique<DebugVehicle>(DebugVehicle(this, assets));
		}

		preview_veh = nullptr;

		if (hover_pos) {
			// place at hover pos if anything hovered and make visible (preview_veh gets rendered)
			
			// recreate SimVehicle used to render vehicle at arbitrary position 
			next_vehicle->sim = std::make_unique<SimVehicle>();
			next_vehicle->sim->path = &next_vehicle->path;

			next_vehicle->sim->init_pos(*hover_pos, next_vehicle->asset);

			// show vehicle only when hover valid (but keep next_vehicle)
			preview_veh = next_vehicle.get();
			
			if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {

				// store as active debug vehicle
				vehicles.push_back(std::move(next_vehicle));

				// create new vehicle to place next frame
				next_vehicle = nullptr;
				preview_veh = nullptr;
			}
		}
	}
	void deselect () {
		next_vehicle = nullptr;
		preview_veh = nullptr;
	}

	void update () {

	}
};

struct Network {
	SERIALIZE(Network, settings, _stay_time);

	void mem_use (MemUse& mem) {
		mem.add("Network", sizeof(*this));
		for (auto& i : nodes) i->mem_use(mem);
		for (auto& i : segments) i->mem_use(mem);
	}

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;

	Metrics metrics;
	Settings settings;

	DebugVehicles debug_vehicles;
	
	int _dijk_iter = 0;
	int _dijk_iter_dupl = 0;
	int _dijk_iter_lanes = 0;

	int active_vehicles = 0;

	// Just an experiment for now
	float _lane_switch_chance = 0.25f;
	float _stay_time = 5*60;

	void imgui () {
		ImGui::Text("Active Vehicles: %5d", active_vehicles);

		metrics.imgui();
		settings.imgui();

		ImGui::SliderFloat("lane_switch_chance", &_lane_switch_chance, 0, 1);
		_lane_switch_chance = clamp(_lane_switch_chance, 0.0f, 1.0f);

		ImGui::DragFloat("stay_time", &_stay_time, 0);
	}

	int pathing_count;

	void simulate (App& app);
	void draw_debug (App& app, View3D& view);
	
	inline Segment* find_nearest_segment (float3 pos) const {
		Segment* nearest_seg = nullptr;
		float min_dist = INF;

		for (auto& seg : segments) {
			float dist = seg->distance_to_point(pos);
			if (dist < min_dist) {
				min_dist = dist;
				nearest_seg = seg.get();
			}
		}

		return nearest_seg;
	}
};

}; // namespace network
