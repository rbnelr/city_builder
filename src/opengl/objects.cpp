#include "objects.hpp"
#include "ogl_render.hpp"

namespace ogl {

struct StaticInstances {
	EntityRenderers& rend;
	Textures& textures;

	std::vector<StaticEntity> buildings;
	std::vector<StaticEntity> props;
	std::vector<StaticEntity> lamps;
	std::vector<StaticEntity> traffic_signals;

	std::vector<DefferedPointLightRenderer::LightInstance> lights;

	void reserve () {
		buildings.reserve(4096);
		props.reserve(4096);
		lamps.reserve(4096);
		traffic_signals.reserve(512);
		lights.reserve(4096);
	}

		
	void _push_light (float3x4 const& matrix, PointLight& light) {
		auto* inst = push_back(lights, 1);
			
		inst->pos = matrix * light.pos;
		inst->radius = light.radius;
		inst->dir = (float3x3)matrix * light.dir;
		inst->cone.x = cos(light.cone.x);
		inst->cone.y = cos(light.cone.y);
		inst->col = light.col * light.strength;
	}
	void _push_base_prop (std::vector<StaticEntity>& vec, PropAsset* prop, float3 pos, float rot) {
		auto* inst = push_back(vec, 1);

		inst->mesh_id = rend.prop_meshes.asset2mesh_id[prop];
		inst->tex_id = textures.bindless_textures[prop->tex_filename];
		inst->pos = pos;
		inst->rot = rot;
		inst->tint = 1;

		auto matrix = obj_transform(pos, rot);
		for (auto& light : prop->lights) {
			_push_light(matrix, light);
		}
	}
	void push_prop (PropAsset* prop, float3 pos, float rot) {
		_push_base_prop(props, prop, pos, rot);
	}
	void push_lamp (PropAsset* prop, float3 pos, float rot) {
		_push_base_prop(lamps, prop, pos, rot);
	}
	void push_traffic_signal (PropAsset* prop, float3 pos, float rot) {
		_push_base_prop(traffic_signals, prop, pos, rot);
	}
};

struct Mesher {
	EntityRenderers& entity_renders;
	NetworkRenderer& network_render;
	DecalRenderer&   decal_render;
	ClippingRenderer& clip_render;
	App            & app;
	Textures       & textures;
	
	EntityRenderers::StaticInstances static_inst;

	Mesher (EntityRenderers& entity_renderers,
	        NetworkRenderer& network_renderer,
	        DecalRenderer& decal_renderer,
	        ClippingRenderer& clip_render,
	        App            & app,
			Textures       & textures):
		entity_renders{entity_renderers}, network_render{network_renderer}, decal_render{decal_renderer}, clip_render{clip_render}, app{app}, textures{textures},
		static_inst{ entity_renderers, textures } {}
	
	Mesh<NetworkRenderer::Vertex, uint32_t> network_mesh;
	std::vector<DecalRenderer::Instance> decals;
	std::vector<ClippingRenderer::Instance> clippings;
	
	void place_segment_line_props (network::Segment& seg, NetworkAsset::Streetlight& light) {
		float y = 0;
		while (y < seg._length) {
			// TODO: introduce curved segments, then rework this!
			float3 dir = seg.node_b->pos - seg.node_a->pos;
			float3 forw = normalizesafe(dir);
			float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
			float3 up = cross(right, forw);
			float rot = light.rot + angle2d(forw);
			float3 shift = light.shift + float3(0,y,0);
			float3 pos = seg.pos_a + right * shift.x + forw * shift.y + up * shift.z;

			static_inst.push_lamp(light.prop.get(), pos, rot);

			y += light.spacing;
		}
	}
	void place_traffic_light (network::Segment& seg, network::Node* node) {
		auto* prop = seg.asset->traffic_light_prop.get();

		// TODO: don't place mask if no incoming lanes (eg. because of one way road)
		
		auto* other_node = seg.get_other_node(node);
		float3 dir = node->pos - other_node->pos;
		float3 forw = normalizesafe(dir);
		float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
		float3 up = cross(right, forw);
		float rot = angle2d(forw);

		float2 shift = seg.asset->traffic_light_shift;
		float3 pos = seg.pos_for_node(node) + right * shift.x + forw * shift.y;

		static_inst.push_prop(prop->mast_prop.get(), pos, rot);

		for (auto in_lane : seg.in_lanes(node)) {
			auto& layout = seg.get_lane_layout(&in_lane.get());
			float x = (seg.get_dir_to_node(node) == LaneDir::FORWARD ? +layout.shift : -layout.shift) - shift.x;

			float3 local_pos = prop->get_signal_pos(x);
			float3 spos = pos + right * local_pos.x + forw * local_pos.y + up * local_pos.z;

			static_inst.push_traffic_signal(prop->signal_prop.get(), spos, rot);
		}
	}

	// Could be instanced in smart ways, by computing beziers on gpu
	// for ex. for road segments: store 'cross section' per road time and extrude it along bezier during instancing on gpu
	// for intersections: greatly depends on how intersecyions will look, but could probably do similar
	// but if properly lodded (via chunks probably) should run fine and unlikely take too much vram

	// TODO: make sure to do proper indexing?
	// OR actually load road mesh as asset and morpth it into shape

	// TODO: fix tangents being wrong sometimes?
	
	float curbstone_w = 0.25f;

	// negative numbers are for non uv mappes (worldspace) textures
	int asphalt_tex_id;
	int curb_tex_id;
	int sidewalk_tex_id;

	float2 curb_tex_tiling = float2(2,0);

	float streetlight_spacing = 10;
	
	float stopline_width  = 1.0f;

	static constexpr float2 no_uv = 0;

	float3 norm_up = float3(0,0,1);
	float3 tang_up = float3(1,0,0);

	void mesh_segment (network::Segment& seg) {
		float width = seg.asset->width;
		
		float3 dir = seg.node_b->pos - seg.node_a->pos;

		float3 forw = normalizesafe(dir);
		float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
		float3 up = cross(right, forw);

		float3 tang = forw;
		
		typedef NetworkRenderer::Vertex V;
		
		float road_z = network::ROAD_Z;
		float sidw_z = 0;

		auto extrude = [&] (V const& l, V const& r, float2 uv_tiling=0) {
			V l0 = l;
			V l1 = l;
			V r0 = r;
			V r1 = r;

			l0.pos = seg.pos_a + right * l.pos.x + forw * l.pos.y + up * l.pos.z;
			l1.pos = seg.pos_b + right * l.pos.x + forw * l.pos.y + up * l.pos.z;

			r0.pos = seg.pos_a + right * r.pos.x + forw * r.pos.y + up * r.pos.z;
			r1.pos = seg.pos_b + right * r.pos.x + forw * r.pos.y + up * r.pos.z;

			if (uv_tiling.x > 0) {
				l1.uv.x += seg._length / uv_tiling.x;
				r1.uv.x += seg._length / uv_tiling.x;
			}
			if (uv_tiling.y > 0) {
				l1.uv.y += seg._length / uv_tiling.y;
				r1.uv.y += seg._length / uv_tiling.y;
			}

			network_mesh.push_quad(l0, r0, r1, l1);
		};
		
		float3 diag_right = (up + right) * SQRT_2/2;
		float3 diag_left  = (up - right) * SQRT_2/2;

		V sL0  = { float3(         -width*0.5f              , 0, sidw_z), up,         tang, no_uv, sidewalk_tex_id };
		V sL1  = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidw_z), up,         tang, no_uv, sidewalk_tex_id };
		V sL1b = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidw_z), up,         tang, float2(0,0), curb_tex_id };
		V sL2  = { float3(seg.asset->sidewalkL              , 0, sidw_z), diag_right, tang, float2(0,0.6f), curb_tex_id };
		V sL3  = { float3(seg.asset->sidewalkL              , 0, road_z), right,      tang, float2(0,1), curb_tex_id };
		V r0   = { float3(seg.asset->sidewalkL              , 0, road_z), up, tang, no_uv, asphalt_tex_id };
		V r1   = { float3(seg.asset->sidewalkR              , 0, road_z), up, tang, no_uv, asphalt_tex_id };
		V sR0  = { float3(seg.asset->sidewalkR              , 0, road_z), -right,    tang, float2(0,1), curb_tex_id };
		V sR1  = { float3(seg.asset->sidewalkR              , 0, sidw_z), diag_left, tang, float2(0,0.6f), curb_tex_id };
		V sR2b = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidw_z), up,        tang, float2(0,0), curb_tex_id };
		V sR2  = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidw_z), up,        tang, no_uv, sidewalk_tex_id };
		V sR3  = { float3(         +width*0.5f              , 0, sidw_z), up,        tang, no_uv, sidewalk_tex_id };

		extrude(sL0 , sL1 );
		extrude(sL1b, sL2 , curb_tex_tiling);
		extrude(sL2 , sL3 , curb_tex_tiling);
		extrude(r0  , r1  );
		extrude(sR0 , sR1 , curb_tex_tiling);
		extrude(sR1 , sR2b, curb_tex_tiling);
		extrude(sR2 , sR3 );
		
		for (auto& line : seg.asset->line_markings) {
			int tex_id = textures.bindless_textures[
				line.type == LineMarkingType::LINE ? "misc/line" : "misc/stripe"
			];
			
			float width = line.scale.x;

			float uv_len = seg._length / (line.scale.y*4); // *4 since 1-4 aspect ratio
			uv_len = max(round(uv_len), 1.0f); // round to avoid stopping in middle of stripe

			DecalRenderer::Instance decal;
			decal.pos = (seg.pos_a + seg.pos_b)*0.5f + float3(right * line.shift.x, 0);
			decal.rot = angle2d(forw) - deg(90);
			decal.size = float3(width, seg._length, 1);
			decal.tex_id = tex_id;
			decal.uv_scale = float2(1, uv_len);
			decal.col = 1;
			decals.push_back(decal);
		}

		for (auto& streetlight : seg.asset->streetlights) {
			place_segment_line_props(seg, streetlight);
		}
	}

	void mesh_node (network::Node* node) {
		struct SegInfo {
			float2 pos;
			float2 forw;
			float2 right;
			NetworkAsset* asset;
		};
		auto calc_seg_info = [&] (network::Segment* seg) {
			SegInfo info;

			auto* other = seg->get_other_node(node);
			info.forw = normalizesafe(node->pos - other->pos);
			info.right = rotate90(-info.forw);

			info.pos = seg->pos_for_node(node);

			info.asset = seg->asset;

			return info;
		};

		auto get_bez = [] (float2 a, float2 a_dir, float2 b, float2 b_dir) {
			network::Line la = { float3(a - a_dir,0), float3(a,0) };
			network::Line lb = { float3(b,0), float3(b - b_dir,0) };
			return network::calc_curve(la, lb);
		};

		int count = (int)node->segments.size();
		for (int i=0; i<count; ++i) {
			//network::Segment* l = node->segments[((i-1)+count) % count];
			network::Segment* s = node->segments[i];
			network::Segment* r = node->segments[(i+1) % count];

			auto si = calc_seg_info(s);
			auto ri = calc_seg_info(r);

			float2 sidewalk_Ra0  = si.pos + si.right * si.asset->sidewalkR;
			float2 sidewalk_Rb0  = si.pos + si.right * si.asset->width*0.5f;

			float2 sidewalk_Ra1  = ri.pos + ri.right * ri.asset->sidewalkL;
			float2 sidewalk_Rb1  = ri.pos - ri.right * ri.asset->width*0.5f;

			float2 sidewalk_RR = ri.pos + ri.right * ri.asset->sidewalkL;

			auto sidewalk_Ra = get_bez(sidewalk_Ra0, si.forw, sidewalk_Ra1, ri.forw);
			auto sidewalk_Rb = get_bez(sidewalk_Rb0, si.forw, sidewalk_Rb1, ri.forw);

			//sidewalk_Ra.dbg_draw(app.view, 0, 4, lrgba(1,0,0,1));
			//sidewalk_Rb.dbg_draw(app.view, 0, 4, lrgba(0,1,0,1));

			float2 segL = si.pos + si.right * si.asset->sidewalkL;
			float2 segR = si.pos + si.right * si.asset->sidewalkR;

			float road_z = network::ROAD_Z;
			float sidw_z = 0;

			typedef NetworkRenderer::Vertex V;
			V nodeCenter = { node->pos + float3(0, 0, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg0       = { float3(segL, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg1       = { float3(segR, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };

			int res = 10;
			for (int i=0; i<res; ++i) {
				float t0 = (float)(i  ) / (res);
				float t1 = (float)(i+1) / (res);

				float2 a0 = sidewalk_Ra.eval(t0).pos;
				float2 a1 = sidewalk_Ra.eval(t1).pos;
				float2 b0 = sidewalk_Rb.eval(t0).pos;
				float2 b1 = sidewalk_Rb.eval(t1).pos;

				V sa0g = { float3(a0, road_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1g = { float3(a1, road_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa0  = { float3(a0, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1  = { float3(a1, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb0  = { float3(b0, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb1  = { float3(b1, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				
				V sa0gA = { float3(a0, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
				V sa1gA = { float3(a1, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };

				network_mesh.push_quad(sa0, sb0, sb1, sa1);
				network_mesh.push_quad(sa0g, sa0, sa1, sa1g);
				network_mesh.push_tri(sa0gA, sa1gA, nodeCenter);
			}

			network_mesh.push_tri(seg0, seg1, nodeCenter);
		}
		
		if (node->traffic_light) {
			for (auto& seg : node->segments) {
				place_traffic_light(*seg, node);
			}
		}
	}

	void remesh_network () {
		ZoneScoped;
		
		for (auto& seg : app.network.segments) {
			mesh_segment(*seg);

			auto v = seg->clac_seg_vecs();
			
			{
				float3 forw = normalizesafe(seg->pos_b - seg->pos_a);
				float ang = angle2d((float2)forw);

				float3 center = (seg->pos_a + seg->pos_b) * 0.5f;
				float3 size;
				size.x = distance(seg->pos_b, seg->pos_a) + 15.0f*2; // TODO: this will get done better with better road meshing
				size.y = seg->asset->width;
				size.z = 1.0f + 10.0f; // 10 meters above ground;
				float offs_z = -1.0f + size.z/2;

				ClippingRenderer::Instance clip;
				clip.pos = (seg->pos_a + seg->pos_b) * 0.5f;
				clip.pos.z += offs_z;
				clip.rot = ang;
				clip.size = size;
				clippings.push_back(clip);
			}

			for (laneid_t id=0; id<seg->num_lanes(); ++id) {
				auto seg_lane = network::SegLane{ seg.get(), id };
				auto& lane = seg->lanes[id];
				auto& lane_asset = seg->get_lane_layout(&lane);

				auto li = seg_lane.clac_lane_info();
				// TODO: this will be a bezier
				float3 forw = normalizesafe(li.b - li.a);
				//float3 right = float3(rotate90(-forw), 0);
				float ang = angle2d((float2)forw) - deg(90);

				{ // push turn arrow
					float2 size = float2(1, 1.5f) * lane_asset.width;

					float3 pos = li.b;
					pos -= forw * size.y*0.75f;

					auto filename = textures.turn_arrows[(int)lane.allowed_turns - 1];
					int tex_id = textures.bindless_textures[filename];

					DecalRenderer::Instance decal;
					decal.pos = pos;
					decal.rot = ang;
					decal.size = float3(size, 1);
					decal.tex_id = tex_id;
					decal.uv_scale = 1;
					decal.col = 1;
					decals.push_back(decal);
				}
				
				auto stop_line = [&] (float3 base_pos, float l, float r, int dir, bool type) {
					int tex_id = textures.bindless_textures[type ? "misc/shark_teeth" : "misc/line"];
					
					float width = type ? stopline_width*1.5f : stopline_width;
					float length = r - l;
					
					float uv_len = length / (width*2); // 2 since texture has 1-2 aspect ratio
					uv_len = max(round(uv_len), 1.0f); // round to avoid stopping in middle of stripe

					DecalRenderer::Instance decal;
					decal.pos = base_pos + float3(v.right * ((r+l)*0.5f), 0);
					decal.rot = angle2d(v.right) + deg(90) * (dir == 0 ? -1 : +1);
					decal.size = float3(width, length, 1);
					decal.tex_id = tex_id;
					decal.uv_scale = float2(1, uv_len);
					decal.col = 1;
					decals.push_back(decal);
				};

				if (lane_asset.direction == LaneDir::FORWARD) {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_b, l, r, 0, lane.yield);
				}
				else {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_a, l, r, 1, lane.yield);
				}
			}
		}

		for (auto& node : app.network.nodes) {
			mesh_node(node.get());
		}
	}

	void push_buildings () {
		ZoneScoped;

		static_inst.buildings.resize(app.entities.buildings.size());

		for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
			auto& entity = app.entities.buildings[i];
			auto& inst = static_inst.buildings[i];
			
			inst.mesh_id = entity_renders.building_meshes.asset2mesh_id[entity->asset];
			inst.tex_id = textures.bindless_textures[entity->asset->tex_filename];
			inst.pos = entity->pos;
			inst.rot = entity->rot;
			inst.tint = 1;
		}
	}

	void remesh (App& app) {
		ZoneScoped;

		static_inst.reserve();
		
		// get diffuse texture id (normal is +1)
		// negative means worldspace uv mapped
		asphalt_tex_id  = -textures.bindless_textures[textures.asphalt];
		sidewalk_tex_id = -textures.bindless_textures[textures.pavement];
		curb_tex_id     =  textures.bindless_textures[textures.curb];

		push_buildings();

		remesh_network();
		
		entity_renders.upload_static_instances(static_inst);
		
		network_render.upload(network_mesh);
		decal_render.upload(decals);
		clip_render.upload(clippings);
	}
};

void OglRenderer::upload_static_instances (App& app) {
	Mesher mesher{ entity_render, network_render, decal_render, clip_render, app, textures };
	mesher.remesh(app);
}

// Match order in Mesher::remesh_network->mesh_node
void OglRenderer::update_dynamic_traffic_signals (Network& net) {
	std::vector<DynamicTrafficSignal> signal_colors;
	signal_colors.reserve(512);

	for (auto& node : net.nodes) {
		if (node->traffic_light) {
			assert(node->traffic_light->behavior);
			node->traffic_light->behavior->push_signal_colors(node.get(), signal_colors);
		}
	}

	entity_render.traffic_signals.upload<1>(signal_colors, true);
}

void OglRenderer::upload_vehicle_instances (App& app, View3D& view) {
	ZoneScoped;
		
	std::vector<DynamicVehicle> instances;
	instances.reserve(4096); // not all persons have active vehicle, don't overallocate

	for (auto& entity : app.entities.persons) {
		if (entity->vehicle) {
			uint32_t instance_idx = (uint32_t)instances.size();
			auto& instance = instances.emplace_back();

			update_vehicle_instance(instance, *entity, instance_idx, view, app.input.real_dt);
		}
	}

	entity_render.vehicles.upload<0>(instances, true);
}
void OglRenderer::update_vehicle_instance (DynamicVehicle& instance, Person& entity, int i, View3D& view, float dt) {
	auto& bone_mats = entity.owned_vehicle->bone_mats;
		
	int tex_id = textures.bindless_textures[entity.owned_vehicle->tex_filename];

	auto vehicle_hash = (uint32_t)hash((size_t)entity.vehicle.get());
	float rand1 = (float)vehicle_hash / (float)UINT_MAX;

	bool blinker_on = entity.vehicle->update_blinker(rand1, dt);

	// TODO: network code shoud ensure length(dir) == CAR_SIZE
	float3 center;
	float ang;
	entity.vehicle->calc_pos(&center, &ang);

	instance.mesh_id = entity_render.vehicle_meshes.asset2mesh_id[entity.owned_vehicle];
	instance.instance_id = i;
	instance.tex_id = tex_id;
	instance.pos = center;
	instance.tint = entity.col;

	instance.glow.x = 255;
	instance.glow.y = entity.vehicle->brake_light > 0.5f ? 255 : 0;
	instance.glow.z = entity.vehicle->blinker < 0.0f && blinker_on ? 255 : 0;
	instance.glow.w = entity.vehicle->blinker > 0.0f && blinker_on ? 255 : 0;
		
	float3x3 heading_rot = rotate3_Z(ang);

	// skip expensive bone matricies computation when too far away
	float anim_LOD_dist = 250;
	if (length_sqr(center - view.cam_pos) > anim_LOD_dist*anim_LOD_dist) {
		for (auto& mat : instance.bone_rot) {
			mat = float4x4(heading_rot);
		}
		return;
	}

	// LOD this since currently bone matricies are too slow in debug mode
	// TODO: find a reasonable way to optimize this? Can't think of a clean way of doing matrix math much faster (especially in unoptimized mode)
	//    -> quaternions
	//  move to gpu? could easily do this in compute shader by passing wheel_roll, turn_curv, suspension_ang per instance as well
	//  could simply keep matricies in instance buffer and have compute shader read/write the vbo in different places? I think that's safe
	//  could also just split out the matrices into other buffer
	//  the question is if this is premature optimization, animated persons would be different again, and might want to move other code on gpu in the future
				
	auto set_bone_rot = [&] (int boneID, float3x3 const& bone_rot) {
		instance.bone_rot[boneID] = float4x4(heading_rot) *
			(bone_mats[boneID].bone2mesh * float4x4(bone_rot) * bone_mats[boneID].mesh2bone);
	};

	float wheel_ang = entity.vehicle->wheel_roll * -deg(360);
	float3x3 roll_mat = rotate3_Z(wheel_ang);

	float rear_axle_x = (bone_mats[VBONE_WHEEL_BL].bone2mesh * float4(0,0,0,1)).x;
				
	auto get_wheel_turn = [&] (int boneID) {
		// formula for ackerman steering with fixed rear axle
		// X is forw, Y is left
		float2 wheel_pos2d = (float2)(bone_mats[boneID].bone2mesh * float4(0,0,0,1));
		float2 wheel_rel2d = wheel_pos2d - float2(rear_axle_x, 0);

		float c = entity.vehicle->turn_curv;
		float ang = -atanf((c * wheel_rel2d.x) / (c * -wheel_rel2d.y - 1.0f));

		return rotate3_Y(ang);
	};

	float3x3 base_rot = rotate3_X(entity.vehicle->suspension_ang.x) * rotate3_Z(-entity.vehicle->suspension_ang.y);
	set_bone_rot(VBONE_BASE, base_rot);


	set_bone_rot(VBONE_WHEEL_FL, get_wheel_turn(VBONE_WHEEL_FL) * roll_mat);
	set_bone_rot(VBONE_WHEEL_FR, get_wheel_turn(VBONE_WHEEL_FR) * roll_mat);

	set_bone_rot(VBONE_WHEEL_BL, roll_mat);
	set_bone_rot(VBONE_WHEEL_BR, roll_mat);
}

} // namespace ogl